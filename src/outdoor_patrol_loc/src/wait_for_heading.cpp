// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// wait_for_heading — block until the GNSS heading is actually being published,
// then exit 0 so a launch file can start the next node.
//
// `navsat_transform` computes the UTM->map transform ONCE, from the first
// moment it holds a GPS fix, an odometry pose and an IMU orientation together,
// and it exposes no `datum` service to re-seed afterwards. Everything it
// publishes for the rest of the session inherits whatever that one computation
// produced.
//
// `um982_driver` deliberately drops the dual-antenna heading while the
// ANT1->ANT2 baseline is unsolved (see the wiki: ANT2 with no signal), so on a
// cold start -- a power-on after a battery swap, say -- `/gnss/heading` is
// simply absent until the receiver reacquires. Starting the localization stack
// in that window means `navsat_transform` initialises against whatever state
// the rest of the stack has drifted into with no absolute yaw, and the result
// is frozen in for the session.
//
// Waiting costs a few seconds on a good day. Not waiting costs a map frame
// that looks healthy on every individual topic and is wrong.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

class WaitForHeading : public rclcpp::Node
{
public:
  WaitForHeading()
  : rclcpp::Node("wait_for_heading")
  {
    topic_ = declare_parameter<std::string>("topic", "/gnss/heading");
    // More than one message, because a single stale latched sample is not
    // evidence that the receiver is solving the baseline right now.
    required_ = declare_parameter<int>("required_messages", 5);
    // 0 disables the timeout: wait indefinitely. That is the default on
    // purpose -- a stack that has not localized is a visible, safe failure,
    // whereas one that silently localizes into a wrong frame is neither.
    timeout_s_ = declare_parameter<double>("timeout_s", 0.0);
    report_period_s_ = declare_parameter<double>("report_period_s", 10.0);

    start_ = now();
    last_report_ = start_;

    // The publisher is best-effort; a reliable subscription would never match.
    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_, qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr) {
        ++count_;
        if (count_ >= required_) {
          RCLCPP_INFO(
            get_logger(),
            "GNSS heading is live on %s (%d messages); starting navsat_transform.",
            topic_.c_str(), count_);
          done_ = true;
        }
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {this->tick();});

    RCLCPP_INFO(
      get_logger(),
      "Waiting for %d messages on %s before navsat_transform starts "
      "(timeout %s).",
      required_, topic_.c_str(),
      timeout_s_ > 0.0 ? (std::to_string(timeout_s_) + " s").c_str() : "none");
  }

  bool timed_out() const {return timed_out_;}

private:
  void tick()
  {
    if (done_) {
      rclcpp::shutdown();
      return;
    }
    const auto elapsed = (now() - start_).seconds();
    if (timeout_s_ > 0.0 && elapsed >= timeout_s_) {
      RCLCPP_ERROR(
        get_logger(),
        "No GNSS heading on %s after %.0f s (%d/%d messages). Not starting "
        "navsat_transform: it would fix the map frame using a heading that "
        "does not exist. Check ANT2 -- the secondary antenna solves the "
        "heading baseline.",
        topic_.c_str(), elapsed, count_, required_);
      timed_out_ = true;
      rclcpp::shutdown();
      return;
    }
    if ((now() - last_report_).seconds() >= report_period_s_) {
      last_report_ = now();
      RCLCPP_WARN(
        get_logger(),
        "Still waiting for GNSS heading on %s (%d/%d after %.0f s). The "
        "dual-antenna baseline is probably unsolved; check ANT2.",
        topic_.c_str(), count_, required_, elapsed);
    }
  }

  std::string topic_;
  int required_{5};
  double timeout_s_{0.0};
  double report_period_s_{10.0};
  int count_{0};
  bool done_{false};
  bool timed_out_{false};
  rclcpp::Time start_;
  rclcpp::Time last_report_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaitForHeading>();
  rclcpp::spin(node);
  const bool failed = node->timed_out();
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return failed ? 1 : 0;
}
