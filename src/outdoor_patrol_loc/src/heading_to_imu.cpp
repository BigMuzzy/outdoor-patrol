// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// Adapt the UM982 dual-antenna heading to a yaw-only sensor_msgs/Imu.
//
// The receiver publishes its dual-antenna baseline heading as a
// geometry_msgs/QuaternionStamped, which neither robot_localization's EKF nor
// navsat_transform_node can consume directly. This node republishes it as a
// sensor_msgs/Imu carrying ONLY an absolute yaw orientation (angular velocity
// and linear acceleration are marked unavailable per the Imu convention,
// covariance[0] = -1), so it can stand in for an IMU's heading until the real
// IMU lands at M2 (interim per ADR-012).
//
// A single `yaw_offset` (plus `invert`) maps the receiver's heading convention
// + antenna mounting into a REP-103 yaw (0 = East, CCW positive):
//
//     yaw_rep103 = yaw_offset + sign * yaw_in
//
// TBD: set `yaw_offset` / `invert` once the heading convention and the
// antenna-baseline mounting angle are confirmed (integration plan items 2/3).

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace outdoor_patrol_loc
{

class HeadingToImu : public rclcpp::Node
{
public:
  HeadingToImu()
  : rclcpp::Node("heading_to_imu")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/um982_driver/heading");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/gnss/heading");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    // TBD: receiver convention (compass->ENU) + antenna mount, radians.
    yaw_offset_ = declare_parameter<double>("yaw_offset", 0.0);
    // TBD: set true if the receiver heading increases clockwise.
    sign_ = declare_parameter<bool>("invert", false) ? -1.0 : 1.0;
    // 1-sigma heading uncertainty (deg) -> orientation yaw covariance.
    const double sd = declare_parameter<double>("yaw_stddev_deg", 1.0) * M_PI / 180.0;
    yaw_var_ = sd * sd;

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
    sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      input_topic, 10,
      std::bind(&HeadingToImu::callback, this, std::placeholders::_1));
  }

private:
  // Planar yaw (rad) from a full quaternion.
  static double yaw_from_quat(double x, double y, double z, double w)
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  void callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    const auto & q = msg->quaternion;
    const double yaw = yaw_offset_ + sign_ * yaw_from_quat(q.x, q.y, q.z, q.w);

    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;
    out.orientation.z = std::sin(yaw / 2.0);
    out.orientation.w = std::cos(yaw / 2.0);
    // Only yaw is observed; roll/pitch get a large variance.
    out.orientation_covariance = {
      1.0e6, 0.0, 0.0,
      0.0, 1.0e6, 0.0,
      0.0, 0.0, yaw_var_};
    // Angular velocity / linear acceleration unavailable: leading -1 tells
    // consumers to ignore them entirely.
    out.angular_velocity_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    out.linear_acceleration_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    pub_->publish(out);
  }

  std::string frame_id_;
  double yaw_offset_ {0.0};
  double sign_ {1.0};
  double yaw_var_ {0.0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_;
};

}  // namespace outdoor_patrol_loc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<outdoor_patrol_loc::HeadingToImu>());
  rclcpp::shutdown();
  return 0;
}
