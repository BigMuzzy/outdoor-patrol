// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// GNSS confidence gate — inflate NavSatFix covariance on a degraded fix.
//
// Interim gate per ADR-001 / ADR-012 (the RTK-status + HDOP subset of OQ-003).
//
// The UM982 driver already folds fix quality (RTK Fixed / Float / DGPS /
// single) and HDOP into the NavSatFix position_covariance (GST 1-sigma when
// fresh, a quality/HDOP heuristic otherwise). So this node gates on the
// reported horizontal sigma — a typed, already-fused signal — instead of
// scraping the discrete RTK flag out of /diagnostics:
//
//   * status == NO_FIX             -> drop (no valid position; the EKF
//                                     dead-reckons on wheel odom over the gap)
//   * horizontal sigma > threshold -> republish with covariance inflated, so
//                                     the global EKF de-weights GNSS *smoothly*
//                                     (no `map` jump — the ADR-001 invariant)
//   * otherwise (RTK-fixed, cm)    -> pass through unchanged
//
// A ~5 cm threshold passes RTK-Fixed only; Float/DGPS/single get inflated. The
// LIO scan-match residual (the third ADR-001 gate condition) is added when LIO
// lands (M4). A discrete RTK-Fixed-vs-Float gate would need a typed status
// topic from the driver; the sigma threshold captures it continuously.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace outdoor_patrol_loc
{

class ConfidenceGate : public rclcpp::Node
{
public:
  ConfidenceGate()
  : rclcpp::Node("confidence_gate")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/um982_driver/fix");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/gnss/fix_gated");
    // Horizontal 1-sigma (m) above which a fix is "degraded". ~5 cm passes
    // RTK-Fixed only; Float/DGPS/single get inflated.
    max_sigma_ = declare_parameter<double>("max_horizontal_sigma_m", 0.05);
    inflation_ = declare_parameter<double>("covariance_inflation", 1000.0);
    reject_no_fix_ = declare_parameter<bool>("reject_on_no_fix", true);

    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(output_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      input_topic, 10,
      std::bind(&ConfidenceGate::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    using NavSatFix = sensor_msgs::msg::NavSatFix;
    using NavSatStatus = sensor_msgs::msg::NavSatStatus;

    const bool no_fix = msg->status.status == NavSatStatus::STATUS_NO_FIX;
    if (no_fix && reject_no_fix_) {
      log_state(true, std::numeric_limits<double>::infinity(), "NO_FIX (dropped)");
      return;
    }

    const double sigma_h = std::sqrt(
      std::max({msg->position_covariance[0], msg->position_covariance[4], 0.0}));
    const bool unknown_cov =
      msg->position_covariance_type == NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    const bool degraded = no_fix || unknown_cov || sigma_h > max_sigma_;

    if (!degraded) {
      pub_->publish(*msg);
      log_state(false, sigma_h, "");
      return;
    }

    NavSatFix out;
    out.header = msg->header;
    out.status = msg->status;
    out.latitude = msg->latitude;
    out.longitude = msg->longitude;
    out.altitude = msg->altitude;
    for (size_t i = 0; i < out.position_covariance.size(); ++i) {
      out.position_covariance[i] = msg->position_covariance[i] * inflation_;
    }
    out.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    pub_->publish(out);
    log_state(true, sigma_h, unknown_cov ? "unknown covariance" : "sigma > threshold");
  }

  void log_state(bool degraded, double sigma_h, const std::string & reason)
  {
    if (have_state_ && degraded == degraded_) {
      return;
    }
    have_state_ = true;
    degraded_ = degraded;
    if (degraded) {
      RCLCPP_WARN(
        get_logger(), "GNSS gate: DEGRADED (%s) -> covariance inflated x%.0f",
        reason.c_str(), inflation_);
    } else {
      RCLCPP_INFO(
        get_logger(), "GNSS gate: OK (sigma=%.3f m) -> pass-through", sigma_h);
    }
  }

  double max_sigma_ {0.05};
  double inflation_ {1000.0};
  bool reject_no_fix_ {true};
  bool have_state_ {false};
  bool degraded_ {false};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

}  // namespace outdoor_patrol_loc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<outdoor_patrol_loc::ConfidenceGate>());
  rclcpp::shutdown();
  return 0;
}
