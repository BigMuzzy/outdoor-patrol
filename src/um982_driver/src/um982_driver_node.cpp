// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/um982_driver_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace um982_driver
{

Um982DriverNode::Um982DriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("um982_driver", options)
{
  declare_parameters();
}

void Um982DriverNode::declare_parameters()
{
  port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = this->declare_parameter<int64_t>("baudrate", 115200);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "gnss_link");
  mode_ = this->declare_parameter<std::string>("mode", "rover");
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring UM982 driver (port=%s, baud=%ld, mode=%s)",
    port_.c_str(), baudrate_, mode_.c_str());

  fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix", 10);
  vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/fix_velocity", 10);
  heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("~/heading", 10);
  nmea_pub_ = create_publisher<nmea_msgs::msg::Sentence>("~/nmea_sentence", 10);
  time_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("~/time_reference", 10);

  rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(
    "rtcm/in", rclcpp::QoS(50),
    std::bind(&Um982DriverNode::on_rtcm, this, std::placeholders::_1));

  RCLCPP_WARN(get_logger(),
    "UM982 driver is a Phase-A scaffold; serial I/O and protocol parsing "
    "are not yet implemented.");
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  fix_pub_->on_activate();
  vel_pub_->on_activate();
  heading_pub_->on_activate();
  nmea_pub_->on_activate();
  time_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "UM982 driver activated (no-op until Phase C).");
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  fix_pub_->on_deactivate();
  vel_pub_->on_deactivate();
  heading_pub_->on_deactivate();
  nmea_pub_->on_deactivate();
  time_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  rtcm_sub_.reset();
  fix_pub_.reset();
  vel_pub_.reset();
  heading_pub_.reset();
  nmea_pub_.reset();
  time_pub_.reset();
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void Um982DriverNode::on_rtcm(const rtcm_msgs::msg::Message::SharedPtr msg)
{
  // Phase C: forward bytes to the serial port.
  RCLCPP_DEBUG(get_logger(), "Received %zu RTCM bytes (dropped: driver stub).",
    msg->message.size());
}

}  // namespace um982_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<um982_driver::Um982DriverNode>(rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
