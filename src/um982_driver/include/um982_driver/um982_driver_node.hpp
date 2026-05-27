// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__UM982_DRIVER_NODE_HPP_
#define UM982_DRIVER__UM982_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rtcm_msgs/msg/message.hpp"

namespace um982_driver
{

/// Lifecycle driver node for the Unicore UM982 RTK GNSS module.
///
/// Phase A: scaffold only — declares parameters, publishers and the RTCM
/// subscription. Serial I/O and protocol parsing are added in Phase C.
class Um982DriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit Um982DriverNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declare_parameters();
  void on_rtcm(const rtcm_msgs::msg::Message::SharedPtr msg);

  // Parameters
  std::string port_;
  int64_t baudrate_{115200};
  std::string frame_id_;
  std::string mode_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
    heading_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::TimeReference>::SharedPtr time_pub_;

  // Subscriptions
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};

}  // namespace um982_driver

#endif  // UM982_DRIVER__UM982_DRIVER_NODE_HPP_
