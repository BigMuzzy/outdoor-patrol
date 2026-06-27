// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef IMU_DRIVER__IMU_DRIVER_NODE_HPP_
#define IMU_DRIVER__IMU_DRIVER_NODE_HPP_

#include <atomic>
#include <map>
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
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "imu_driver/frame_parser.hpp"
#include "imu_driver/messages.hpp"

namespace imu_driver
{

/// Lifecycle driver for Inertial Labs binary-protocol IMUs (KERNEL family and
/// compatible). Streams the "Calibrated HR Data" format and publishes
/// sensor_msgs/Imu + sensor_msgs/Temperature in the device's own sensor frame.
class ImuDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ImuDriverNode(const rclcpp::NodeOptions & options);
  ~ImuDriverNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declare_parameters();
  bool write_all(const std::vector<uint8_t> & bytes);
  void send_command(Command code);
  void stop_stream();

  /// Synchronously query device identity/health (GetDevInfo + GetBIT) on an
  /// otherwise-idle line, before the streaming read thread starts. Stores
  /// results for diagnostics. Returns false if either reply is not received.
  bool query_device_identity();
  /// Send `cmd` and block up to `timeout_ms` for a frame whose data_id matches
  /// `expected_id`, parsing and storing it. Used only during configuration.
  bool wait_for_response(int fd, Command cmd, uint8_t expected_id, int timeout_ms);

  void read_loop();
  void handle_frame(const Frame & f);
  /// Add one parsed sample to the current block; publishes the block mean once
  /// `publish_every_n_` samples have accumulated.
  void accumulate_sample(const CalibHrData & d);
  /// Publish the mean of the current block and reset it.
  void publish_block();
  void publish_imu(const CalibHrData & d);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Parameters
  std::string port_;
  int64_t baudrate_{115200};
  std::string frame_id_;
  std::string output_format_;
  double gravity_{kGravity};
  bool publish_orientation_{true};
  bool publish_temperature_{true};
  bool query_device_info_{true};
  int64_t publish_every_n_{1};
  double expected_rate_hz_{100.0};
  std::vector<double> orientation_cov_;
  std::vector<double> angular_velocity_cov_;
  std::vector<double> linear_acceleration_cov_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diag_;

  // Serial I/O
  std::atomic<int> fd_{-1};
  std::atomic<bool> running_{false};
  std::thread io_thread_;
  std::mutex write_mutex_;
  FrameParser parser_;
  // Block-average decimation accumulator. Owned solely by the I/O thread: each
  // parsed CalibHR sample is summed here, and once `count` reaches
  // publish_every_n_ the mean is published and the block resets. Averaging
  // (rather than dropping N-1 of every N samples) uses the full device rate to
  // lower gyro/accel white noise by ~sqrt(N) and acts as a cheap anti-alias
  // filter.
  struct SampleBlock
  {
    double angular_velocity[3]{0.0, 0.0, 0.0};
    double linear_acceleration[3]{0.0, 0.0, 0.0};
    double temperature_c{0.0};
    uint16_t usw_raw{0};   ///< OR-accumulated raw USW so no fault bit is lost.
    uint64_t count{0};
    CalibHrData last;      ///< Most recent sample (orientation, counter, etc.).
  };
  SampleBlock block_;

  // State snapshot for diagnostics (guarded by state_mutex_)
  std::mutex state_mutex_;
  rclcpp::Time last_sample_time_;
  size_t samples_total_{0};            ///< CalibHR samples received from the device.
  size_t samples_at_last_diag_{0};
  size_t published_total_{0};          ///< Imu messages actually published (post-decimation).
  size_t published_at_last_diag_{0};
  rclcpp::Time last_diag_time_;
  std::optional<Usw> last_usw_;
  std::optional<double> last_temperature_c_;
  std::optional<DevInfo> dev_info_;
  std::optional<BitStatus> bit_status_;
  size_t unknown_frames_{0};
  std::map<uint8_t, size_t> unknown_ids_;  ///< data_id -> count of unhandled frames.
};

}  // namespace imu_driver

#endif  // IMU_DRIVER__IMU_DRIVER_NODE_HPP_
