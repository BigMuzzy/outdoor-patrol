// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__UM982_DRIVER_NODE_HPP_
#define UM982_DRIVER__UM982_DRIVER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

#include "um982_driver/frame_splitter.hpp"
#include "um982_driver/nmea_parser.hpp"
#include "um982_driver/unicore_parser.hpp"

namespace um982_driver
{

/// Lifecycle driver node for the Unicore UM982 RTK GNSS module.
class Um982DriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit Um982DriverNode(const rclcpp::NodeOptions & options);
  ~Um982DriverNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declare_parameters();
  void on_rtcm(const rtcm_msgs::msg::Message::SharedPtr msg);

  void read_loop();
  void handle_sentence(const Sentence & s);
  void publish_gga(const NmeaGga & gga, const std::string & raw);
  void publish_rmc(const NmeaRmc & rmc);
  void publish_vtg(const NmeaVtg & vtg);
  void publish_ksxt(const KsxtSentence & k);

  void send_init_commands();
  bool write_all(const std::string & cmd);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Parameters
  std::string port_;
  int64_t baudrate_{115200};
  std::string frame_id_;
  std::string heading_frame_id_;
  std::string mode_;
  std::string rover_dynamics_;
  double base_lat_{0.0};
  double base_lon_{0.0};
  double base_height_{0.0};
  double survey_seconds_{60.0};
  double survey_dist_m_{0.0};
  std::string heading2_mode_;
  std::vector<int64_t> rtcm_ids_;
  double rtcm_period_s_{1.0};
  std::string rtcm_out_com_;
  std::vector<std::string> output_messages_;
  double output_period_s_{0.2};
  std::string output_com_;
  double antenna_h_{0.0};
  double antenna_e_{0.0};
  double antenna_n_{0.0};
  bool save_config_{false};
  bool unlogall_on_configure_{true};

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

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diag_;

  // Serial I/O
  std::atomic<int> fd_{-1};
  std::atomic<bool> running_{false};
  std::thread io_thread_;
  std::mutex write_mutex_;
  FrameSplitter splitter_;

  // State snapshot for diagnostics
  std::mutex state_mutex_;
  NmeaFixQuality last_quality_{NmeaFixQuality::kInvalid};
  uint16_t last_num_sats_{0};
  double last_hdop_{0.0};
  std::optional<double> last_correction_age_s_;
  rclcpp::Time last_fix_time_;
  rclcpp::Time last_rtcm_in_time_;
  rclcpp::Time last_gst_time_;
  std::optional<NmeaGst> last_gst_;
  size_t rtcm_bytes_in_{0};
  size_t sentences_in_{0};
  size_t unicore_in_{0};
  size_t bad_checksums_{0};
};

}  // namespace um982_driver

#endif  // UM982_DRIVER__UM982_DRIVER_NODE_HPP_
