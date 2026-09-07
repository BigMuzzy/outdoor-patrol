// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// patrol_mission — drive a recorded teach-and-repeat route with Nav2.
//
// Replaces route_follower's control loop with stock Nav2 (plan Phase 1). What
// is left here is only what Nav2 cannot know:
//
//   1. The route is GEODETIC. Every sample is projected through
//      robot_localization's /fromLL at start-up, exactly as the follower does
//      it, so the mission survives a datum change.
//   2. Nav2 has no opinion about GNSS quality. Above `sigma_stop_m` on the RAW
//      driver fix the goal is cancelled; below `sigma_slow_m` for
//      `resume_clear_cycles` it is re-sent from where it stopped.
//   3. The harness parses a status topic. ~/status mirrors
//      /route_follower/status so score_run.py needs only --status-topic.
//
// Everything else is stock: Hybrid-A* plans, SavitzkyGolaySmoother smooths,
// MPPI drives, velocity_smoother clamps, behavior_server recovers.
//
// NavigateThroughPoses, not the waypoint_follower's FollowGPSWaypoints, which
// would have done the geodetic projection for us: FollowGPSWaypoints runs one
// NavigateToPose per waypoint and comes to a STOP at each. With stations every
// 10 m that is a stop every 10 m, and R3-N requires the longest stop to stay
// under 3 s. NavigateThroughPoses treats intermediate poses as via-points.
//
// This node publishes NO velocity, on any topic. On cancel, Nav2 stops
// commanding; scan_safety's cmd_timeout_s (0.5 s) then emits the single zero
// Twist that actually stops the robot -- the same path the follower relies on.
// Publishing our own zero would mean two writers on /cmd_vel_raw.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "outdoor_patrol_nav/route_goals.hpp"

namespace outdoor_patrol_nav
{

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

// Reported in ~/status. Same vocabulary as route_follower, because
// score_run.py counts 'degraded' cycles by name.
constexpr char kStateDriving[] = "driving";
constexpr char kStateBlocked[] = "blocked";
constexpr char kStateDegraded[] = "degraded";
constexpr char kStateFinished[] = "finished";

/// Horizontal 1-sigma (m) of a NavSatFix, or inf if it cannot be trusted.
///
/// Ported inline from route_file.horizontal_sigma() -- eight lines, and both
/// of its traps are load-bearing:
///
/// **Use the WORSE axis, not just east.** [0] is east variance, [4] north.
/// Measured over a 20-minute RTK-fixed soak at this site north was 1.7x worse
/// than east. confidence_gate tests max(cov[0], cov[4]); the two must agree on
/// what "sigma" means or this node will drive on a fix the gate calls degraded.
///
/// **A missing covariance is not a perfect fix.** position_covariance is
/// all-zero when the type is UNKNOWN, so a naive sqrt(cov[0]) returns 0.0 --
/// flawless, full speed. Same for NO_FIX. Both return inf so callers fail safe.
/// This matters here precisely because we read the RAW driver topic, where such
/// messages are not filtered out the way confidence_gate filters them.
double horizontal_sigma(const sensor_msgs::msg::NavSatFix & msg)
{
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  if (msg.position_covariance_type == NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    return std::numeric_limits<double>::infinity();
  }
  if (msg.status.status < 0) {  // STATUS_NO_FIX
    return std::numeric_limits<double>::infinity();
  }
  const double variance =
    std::max({msg.position_covariance[0], msg.position_covariance[4], 0.0});
  if (!std::isfinite(variance)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(variance);
}

class PatrolMission : public rclcpp::Node
{
public:
  PatrolMission()
  : rclcpp::Node("patrol_mission")
  {
    route_path_ = declare_parameter<std::string>("route_path", "");
    const auto odom_topic =
      declare_parameter<std::string>("odom_topic", "/odometry/global");
    // RAW driver fix, deliberately NOT /gnss/fix_gated. The gate multiplies
    // covariance by 1000 on a degraded fix -- an EKF-weighting device, not a
    // quality metric -- which would turn the sigma_slow/sigma_stop pair below
    // into an on/off cliff at 5 cm. See route_follower.py's note.
    const auto fix_topic =
      declare_parameter<std::string>("fix_topic", "/um982_driver/fix");
    fromll_service_ = declare_parameter<std::string>("fromll_service", "/fromLL");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    // Dense enough that Hybrid-A* returns the centerline on a clear lane,
    // sparse enough that it is not re-planning between neighbouring poses.
    station_spacing_m_ = declare_parameter<double>("station_spacing_m", 10.0);
    laps_ = declare_parameter<double>("laps", 1.0);
    sigma_slow_m_ = declare_parameter<double>("sigma_slow_m", 0.10);
    sigma_stop_m_ = declare_parameter<double>("sigma_stop_m", 0.50);
    fix_timeout_s_ = declare_parameter<double>("fix_timeout_s", 2.0);
    resume_clear_cycles_ = declare_parameter<int>("resume_clear_cycles", 20);
    const double status_period_s = declare_parameter<double>("status_period_s", 0.1);
    fromll_timeout_s_ = declare_parameter<double>("fromll_timeout_s", 2.0);
    behavior_tree_ = declare_parameter<std::string>("behavior_tree", "");
    // A patrol route is a closed loop, so its last station IS its first. Sent
    // as a single NavigateThroughPoses goal that is satisfied before the robot
    // moves: the intermediate poses are via-points and only the FINAL pose is
    // goal-checked, and the final pose is the one the robot is standing on.
    // Splitting the lap into chunks keeps every goal's final pose far from
    // wherever the robot is when that chunk is sent. Expressed as a distance
    // rather than a pose count so that changing station_spacing_m does not
    // silently change the number of chunk boundaries -- and each boundary is
    // a stop, which R3-N's "longest stop < 3 s" gate cares about. 50 m on a
    // 100 m lap is one boundary.
    max_goal_span_m_ = declare_parameter<double>("max_goal_span_m", 50.0);

    rclcpp::QoS qos(10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", qos);
    // Transient-local so run_validation.sh's `ros2 topic echo --once` sees the
    // flag even if it probes a moment after the mission ended.
    finished_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/finished", rclcpp::QoS(1).transient_local());

    // Numeric telemetry for stock tooling. rqt_plot graphs these directly:
    //   ros2 run rqt_plot rqt_plot
    //     /patrol_mission/cross_track_m/data /patrol_mission/speed_mps/data
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("~/speed_mps", 10);
    cross_track_pub_ = create_publisher<std_msgs::msg::Float64>("~/cross_track_m", 10);
    cross_track_max_pub_ =
      create_publisher<std_msgs::msg::Float64>("~/cross_track_max_m", 10);
    sigma_pub_ = create_publisher<std_msgs::msg::Float64>("~/sigma_h_m", 10);

    // The taught route as a latched Path, so RViz's stock Path display shows
    // it whenever the operator connects rather than only if they were
    // watching when it was published.
    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/route", rclcpp::QoS(1).transient_local());

    // /diagnostics, so rqt_robot_monitor shows the mission beside the GNSS,
    // IMU and lidar entries instead of in a separate window of its own.
    diag_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_->setHardwareID("patrol_mission");
    diag_->add("mission", this, &PatrolMission::produce_diagnostics);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        speed_ = msg->twist.twist.linear.x;
        update_cross_track(msg->pose.pose.position.x, msg->pose.pose.position.y);
      });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic, qos,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        sigma_ = horizontal_sigma(*msg);
        last_fix_ = now();
      });

    // Separate groups, and a MultiThreadedExecutor in main(): build_() blocks
    // on the /fromLL response from inside a timer callback, so the response
    // has to be delivered by a different thread or the node deadlocks.
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    fromll_ = create_client<robot_localization::srv::FromLL>(
      fromll_service_, rclcpp::ServicesQoS(), service_group_);
    nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses", service_group_);

    build_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PatrolMission::build, this), timer_group_);
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_period_s),
      std::bind(&PatrolMission::tick, this), timer_group_);
  }

private:
  // -- start-up ------------------------------------------------------------

  void build()
  {
    if (finished_ || goal_active_ || goal_pending_) {
      return;
    }
    if (plan_.empty() && !build_plan()) {
      return;
    }
    // action_server_is_ready() only reports that the action server has been
    // DISCOVERED. A Nav2 lifecycle node creates its action server in
    // configure() and rejects every goal until activate(), so this check
    // cannot tell the two apart -- and for bt_navigator they are ~1.7 s
    // apart, which the first goal lands squarely inside. The readiness check
    // therefore only avoids the pointless wait_for_action_server() below;
    // being accepted is proven by goal_response_callback, which cancels this
    // timer on acceptance and re-arms it on rejection.
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "waiting for the navigate_through_poses action server -- is "
        "bt_navigator active?");
      return;
    }
    // pending_poses_ is non-zero only when we are retrying, and then it is
    // the tail we last tried to send -- resending plan_.size() there would
    // re-drive stations the robot has already passed.
    send_goal(pending_poses_ > 0 ? pending_poses_ : plan_.size());
  }

  /// Project the route and build the goal list. False = try again next tick.
  bool build_plan()
  {
    if (route_.samples.empty() && !load_route()) {
      build_timer_->cancel();
      return false;
    }
    if (!fromll_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "waiting for %s -- navsat_transform must be running to project the "
        "route into the map frame", fromll_service_.c_str());
      return false;
    }

    std::vector<std::array<double, 2>> xy;
    xy.reserve(route_.samples.size());
    for (const auto & sample : route_.samples) {
      auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
      request->ll_point.latitude = sample.lat;
      request->ll_point.longitude = sample.lon;
      request->ll_point.altitude = 0.0;
      auto future = fromll_->async_send_request(request);
      if (future.wait_for(std::chrono::duration<double>(fromll_timeout_s_)) !=
        std::future_status::ready)
      {
        RCLCPP_ERROR(get_logger(), "fromLL timed out; retrying");
        return false;
      }
      const auto point = future.get()->map_point;
      xy.push_back({point.x, point.y});
    }
    // Kept for live cross-track telemetry. This is the DENSE recorded route in
    // the map frame, not the sparse station list -- deviation has to be
    // measured against the path that was actually taught, otherwise it reports
    // the chord error between stations as if it were tracking error.
    route_xy_ = xy;
    // Publish it for RViz while we have it in map coordinates.
    nav_msgs::msg::Path route_path;
    route_path.header.frame_id = map_frame_;
    route_path.header.stamp = now();
    route_path.poses.reserve(xy.size());
    for (const auto & point : xy) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = route_path.header;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.orientation.w = 1.0;
      route_path.poses.push_back(pose);
    }
    route_pub_->publish(route_path);

    const auto indices = subsample_indices(xy, station_spacing_m_, route_.loop);
    std::vector<geometry_msgs::msg::PoseStamped> stations;
    stations.reserve(indices.size());
    for (const auto index : indices) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = xy[index][0];
      pose.pose.position.y = xy[index][1];
      // Recorded yaw is already REP-103 (ENU, CCW from east).
      pose.pose.orientation.z = std::sin(route_.samples[index].yaw * 0.5);
      pose.pose.orientation.w = std::cos(route_.samples[index].yaw * 0.5);
      stations.push_back(pose);
    }

    // Station 0 is where the robot already stands, so the goal list starts at
    // station 1. On a loop, k % n brings it back to station 0 at the end of
    // each lap. An open route cannot be lapped.
    const std::size_t n = stations.size();
    if (route_.loop) {
      const auto total = static_cast<std::size_t>(
        std::max(1L, std::lround(laps_ * static_cast<double>(n))));
      for (std::size_t k = 1; k <= total; ++k) {
        plan_.push_back(stations[k % n]);
      }
    } else {
      if (std::abs(laps_ - 1.0) > 1e-6) {
        RCLCPP_WARN(get_logger(), "laps=%.2f ignored: this route is not a loop", laps_);
      }
      for (std::size_t k = 1; k < n; ++k) {
        plan_.push_back(stations[k]);
      }
    }
    if (plan_.empty()) {
      RCLCPP_ERROR(
        get_logger(), "route subsampled to %zu stations -- nothing to drive", n);
      build_timer_->cancel();
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "route projected: %zu samples -> %zu stations every %.1f m, "
      "%zu goal poses, loop=%s",
      route_.samples.size(), n, station_spacing_m_, plan_.size(),
      route_.loop ? "true" : "false");
    return true;
  }

  bool load_route()
  {
    if (route_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "route_path is required");
      return false;
    }
    try {
      route_ = read_route(route_path_);
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "cannot load %s: %s", route_path_.c_str(), exc.what());
      return false;
    }
    if (!route_.is_base_link()) {
      RCLCPP_ERROR(
        get_logger(),
        "refusing to follow %s: source=%s records the ANTENNA phase centre, "
        "which on this vehicle is 0.42 m right of base_link",
        route_path_.c_str(), route_.source.c_str());
      route_.samples.clear();
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "route: %zu samples, loop=%s, source=%s",
      route_.samples.size(), route_.loop ? "true" : "false", route_.source.c_str());
    return true;
  }

  // -- goal management -----------------------------------------------------

  void send_goal(std::size_t remaining)
  {
    // Only block when the server has not been seen yet. on_result() calls
    // this from the action client's own callback group, where a blocking
    // wait could deadlock against the response it is waiting for.
    if (!nav_client_->action_server_is_ready() &&
      !nav_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(get_logger(), "navigate_through_poses action server never appeared");
      state_ = kStateBlocked;
      return;
    }
    remaining = std::min(remaining, plan_.size());
    if (remaining == 0) {
      finish();
      return;
    }

    // `remaining` counts back from the END of the plan, so the chunk to drive
    // now is the first max_goal_span_m_ worth of that tail. chunk_base_ is
    // what will still be outstanding once this goal succeeds.
    const auto span_poses = static_cast<std::size_t>(
      std::max(1.0, max_goal_span_m_ / std::max(station_spacing_m_, 0.01)));
    const std::size_t chunk = std::min(remaining, span_poses);
    chunk_base_ = remaining - chunk;

    NavigateThroughPoses::Goal goal;
    goal.behavior_tree = behavior_tree_;
    using Diff = decltype(plan_)::difference_type;
    const auto begin = plan_.end() - static_cast<Diff>(remaining);
    goal.poses.assign(begin, begin + static_cast<Diff>(chunk));
    const auto stamp = now();
    for (auto & pose : goal.poses) {
      pose.header.stamp = stamp;
    }

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        // Nav2 counts down the poses of the CURRENT goal only, so add back the
        // chunks not yet sent. A resume after a degraded stop re-sends exactly
        // this many poses off the tail of plan_, so the robot does not
        // re-drive the stations it already passed.
        const int16_t left = std::max<int16_t>(feedback->number_of_poses_remaining, 0);
        poses_remaining_ = chunk_base_ + static_cast<std::size_t>(left);
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        on_result(result);
      };
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        goal_pending_ = false;
        if (!handle) {
          // Nearly always bt_navigator configured-but-not-yet-active. Re-arm
          // the build timer so the next tick tries again, and leave started_
          // alone: tick() gates status publication on it, and a resume that
          // is briefly rejected must not stop the status stream.
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "navigate_through_poses rejected the goal -- bt_navigator is "
            "probably not active yet; retrying");
          goal_active_ = false;
          build_timer_->reset();
          return;
        }
        goal_handle_ = handle;
        build_timer_->cancel();
      };

    poses_remaining_ = remaining;
    pending_poses_ = remaining;
    goal_pending_ = true;
    goal_active_ = true;
    state_ = kStateDriving;
    started_ = true;
    nav_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "goal sent: %zu poses remaining", remaining);
  }

  void on_result(const GoalHandle::WrappedResult & result)
  {
    goal_active_ = false;
    goal_handle_.reset();
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (chunk_base_ > 0) {
          // Lap not over -- this was only one chunk of it. Send the next one
          // straight from here so the robot barely stops at the boundary.
          RCLCPP_INFO(
            get_logger(), "chunk complete, %zu poses left in the lap", chunk_base_);
          send_goal(chunk_base_);
        } else {
          finish();
        }
        break;
      case rclcpp_action::ResultCode::CANCELED:
        // Expected: this is our own degraded-GNSS cancel. tick() re-sends.
        RCLCPP_INFO(get_logger(), "goal cancelled with %zu poses left", poses_remaining_);
        break;
      default:
        // Nav2 gave up: no plan through the corridor, or the progress checker
        // aborted. Phase 4 turns this into a retreat; for now it is a stop,
        // which is the same fallback the follower had.
        RCLCPP_ERROR(get_logger(), "navigation aborted with %zu poses left", poses_remaining_);
        state_ = kStateBlocked;
        break;
    }
  }

  void cancel_goal()
  {
    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
    }
    goal_active_ = false;
  }

  void finish()
  {
    if (finished_) {
      return;
    }
    finished_ = true;
    state_ = kStateFinished;
    publish_status();
    std_msgs::msg::Bool flag;
    flag.data = true;
    finished_pub_->publish(flag);
    RCLCPP_INFO(get_logger(), "route complete");
  }

  // -- control -------------------------------------------------------------

  void tick()
  {
    // Nothing is published before the first goal: score_run.py opens its
    // scoring window at the first status message, and a start-up stretch of
    // 'degraded' would drag that window back over the sim's settling time.
    if (!started_ || finished_) {
      return;
    }

    double sigma = sigma_;
    if (last_fix_.nanoseconds() == 0 ||
      (now() - last_fix_).seconds() > fix_timeout_s_)
    {
      sigma = std::numeric_limits<double>::infinity();
    }

    if (state_ == kStateDegraded) {
      if (sigma < sigma_slow_m_) {
        ++clear_cycles_;
        if (clear_cycles_ >= resume_clear_cycles_) {
          RCLCPP_INFO(get_logger(), "fix recovered (sigma %.3f m) -- resuming", sigma);
          send_goal(poses_remaining_);
        }
      } else {
        clear_cycles_ = 0;
      }
    } else if (sigma > sigma_stop_m_ && goal_active_) {
      RCLCPP_WARN(
        get_logger(), "GNSS degraded (sigma %.3f m > %.3f m) -- cancelling the goal",
        sigma, sigma_stop_m_);
      cancel_goal();
      state_ = kStateDegraded;
      clear_cycles_ = 0;
    }

    publish_status();
    publish_telemetry();
  }

  /// Signed-magnitude distance from (x, y) to the taught route polyline.
  ///
  /// Live tracking error, published so it can be watched with stock tools
  /// (rqt_plot on the Float64 topics, rqt_robot_monitor on /diagnostics)
  /// rather than a bespoke panel. It is NOT the number score_run.py reports:
  /// the scorer measures against Gazebo ground truth or the world's own
  /// geometry, whereas this is measured against the recorded route using the
  /// robot's own estimate of where it is. It therefore cannot see a
  /// localization error that moves the route and the robot together, and must
  /// not be used as a pass/fail gate. It is a field instrument.
  void update_cross_track(double x, double y)
  {
    if (route_xy_.size() < 2 || !started_ || finished_) {
      return;
    }
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = route_xy_.size();
    // A loop route closes back to sample 0, so the closing segment is real.
    const std::size_t segments = route_.loop ? n : n - 1;
    for (std::size_t i = 0; i < segments; ++i) {
      const auto & a = route_xy_[i];
      const auto & b = route_xy_[(i + 1) % n];
      const double dx = b[0] - a[0];
      const double dy = b[1] - a[1];
      const double len2 = dx * dx + dy * dy;
      double t = 0.0;
      if (len2 > 1e-12) {
        t = ((x - a[0]) * dx + (y - a[1]) * dy) / len2;
        t = std::clamp(t, 0.0, 1.0);
      }
      const double px = a[0] + t * dx;
      const double py = a[1] + t * dy;
      const double d = std::hypot(x - px, y - py);
      if (d < best) {
        best = d;
      }
    }
    if (!std::isfinite(best)) {
      return;
    }
    cross_track_ = best;
    double previous = cross_track_max_.load();
    while (best > previous &&
      !cross_track_max_.compare_exchange_weak(previous, best))
    {
    }
  }

  /// Numeric mirrors of the JSON status, for stock tooling.
  ///
  /// /patrol_mission/status is one std_msgs/String of JSON because
  /// score_run.py parses it that way and R3/R5 comparability depends on the
  /// exact key names. rqt_plot cannot graph a String, so the same values go
  /// out again as Float64 -- no new message package, nothing to install on
  /// the dev box beyond rqt itself.
  void publish_telemetry()
  {
    std_msgs::msg::Float64 value;
    value.data = speed_;
    speed_pub_->publish(value);
    value.data = cross_track_;
    cross_track_pub_->publish(value);
    value.data = cross_track_max_.load();
    cross_track_max_pub_->publish(value);
    value.data = std::isfinite(sigma_) ? sigma_.load() : -1.0;
    sigma_pub_->publish(value);
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    unsigned char level = DiagnosticStatus::OK;
    std::string message = state_;
    if (state_ == kStateBlocked) {
      level = DiagnosticStatus::ERROR;
      message = "blocked: Nav2 could not follow the route";
    } else if (state_ == kStateDegraded) {
      level = DiagnosticStatus::WARN;
      message = "degraded GNSS: goal cancelled, waiting for the fix";
    } else if (!started_) {
      level = DiagnosticStatus::WARN;
      message = "waiting to start";
    }
    stat.summary(level, message);
    stat.add("state", state_);
    stat.add("started", started_.load());
    stat.add("finished", finished_.load());
    stat.add("route_samples", static_cast<int>(route_xy_.size()));
    stat.add("stations_total", static_cast<int>(plan_.size()));
    stat.add("poses_remaining", static_cast<int>(poses_remaining_));
    stat.add("cross_track_m", cross_track_.load());
    stat.add("cross_track_max_m", cross_track_max_.load());
    stat.add("speed_mps", speed_.load());
    if (std::isfinite(sigma_)) {
      stat.add("sigma_h_m", sigma_.load());
    } else {
      stat.add("sigma_h_m", "no fix");
    }
    stat.add("sigma_stop_m", sigma_stop_m_);
  }

  void publish_status()
  {
    // Key names, not the plan document's prose names: score_run.py reads
    // 'state' and 'd_cmd' by name, and 'd_cmd' is an unguarded dict access.
    // There is no commanded lateral offset under Nav2 -- MPPI deviates inside
    // the corridor as it sees fit -- so d_cmd is the constant 0.0 and the
    // scorer's cross_track falls back to the true lateral position measured
    // against the world centerline, which is what it should have been all
    // along. Nothing here is computed from the robot's own estimate.
    std::ostringstream json;
    json.setf(std::ios::fixed);
    json << "{\"state\": \"" << state_ << "\""
         << ", \"d_cmd\": 0.0"
         << ", \"d_target\": null"
         << ", \"blocked\": " << (state_ == kStateBlocked ? "true" : "false")
         << ", \"speed\": " << std::setprecision(3) << speed_
         << ", \"sigma_h\": ";
    if (std::isfinite(sigma_)) {
      json << std::setprecision(4) << sigma_;
    } else {
      json << "null";
    }
    json << ", \"poses_remaining\": " << poses_remaining_
         << ", \"bt_state\": \"" << state_ << "\""
         << ", \"safe_spot\": null"
         << ", \"retreat_attempt\": 0}";

    std_msgs::msg::String message;
    message.data = json.str();
    status_pub_->publish(message);
  }

  std::string route_path_;
  std::string fromll_service_;
  std::string map_frame_;
  std::string behavior_tree_;
  double station_spacing_m_ {10.0};
  double laps_ {1.0};
  double sigma_slow_m_ {0.10};
  double sigma_stop_m_ {0.50};
  double fix_timeout_s_ {2.0};
  double fromll_timeout_s_ {2.0};
  int resume_clear_cycles_ {20};

  RouteData route_;
  std::vector<std::array<double, 2>> route_xy_;
  std::vector<geometry_msgs::msg::PoseStamped> plan_;
  std::size_t poses_remaining_ {0};
  std::size_t pending_poses_ {0};
  std::size_t chunk_base_ {0};
  double max_goal_span_m_ {50.0};
  std::atomic<bool> goal_active_ {false};
  std::atomic<bool> goal_pending_ {false};
  std::atomic<bool> started_ {false};
  std::atomic<bool> finished_ {false};
  int clear_cycles_ {0};
  std::string state_ {kStateDriving};
  std::atomic<double> speed_ {0.0};
  std::atomic<double> sigma_ {std::numeric_limits<double>::infinity()};
  std::atomic<double> cross_track_ {0.0};
  std::atomic<double> cross_track_max_ {0.0};
  rclcpp::Time last_fix_ {0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_max_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sigma_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  std::unique_ptr<diagnostic_updater::Updater> diag_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr fromll_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr build_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace outdoor_patrol_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<outdoor_patrol_nav::PatrolMission>();
  // MultiThreaded, not spin(): see the callback-group comment in the
  // constructor. A single-threaded executor deadlocks on the first /fromLL.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
