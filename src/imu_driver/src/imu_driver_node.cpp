// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "imu_driver/imu_driver_node.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "imu_driver/serial_port.hpp"

namespace imu_driver
{

namespace
{
constexpr uint8_t kDataIdCalibHr = static_cast<uint8_t>(Command::kCalibHR);
constexpr uint8_t kDataIdDevInfo = static_cast<uint8_t>(Command::kGetDevInfo);
constexpr uint8_t kDataIdBit = static_cast<uint8_t>(Command::kGetBIT);

void fill_cov(std::array<double, 9> & dst, const std::vector<double> & src)
{
  if (src.size() == 9) {
    for (size_t i = 0; i < 9; ++i) {
      dst[i] = src[i];
    }
  }
}

// Read available bytes, waiting up to `timeout_ms` for the first byte. Returns
// the number of bytes read, 0 on timeout (line idle), or -1 on error.
ssize_t read_with_timeout(int fd, uint8_t * buf, size_t len, int timeout_ms)
{
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  pfd.revents = 0;
  int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr < 0) {
    return (errno == EINTR) ? 0 : -1;
  }
  if (pr == 0) {
    return 0;  // timeout: no data within the window
  }
  return ::read(fd, buf, len);
}

// Read and discard input until the line stays silent for `quiet_ms`.
void drain_input(int fd, int quiet_ms)
{
  uint8_t buf[1024];
  while (read_with_timeout(fd, buf, sizeof(buf), quiet_ms) > 0) {
  }
}
}  // namespace

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("imu_driver", options)
{
  declare_parameters();
}

ImuDriverNode::~ImuDriverNode()
{
  running_ = false;
  int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void ImuDriverNode::declare_parameters()
{
  port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = this->declare_parameter<int64_t>("baudrate", 115200);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
  output_format_ = this->declare_parameter<std::string>("output_format", "calib_hr");
  gravity_ = this->declare_parameter<double>("gravity", kGravity);
  publish_orientation_ = this->declare_parameter<bool>("publish_orientation", true);
  publish_temperature_ = this->declare_parameter<bool>("publish_temperature", true);
  query_device_info_ = this->declare_parameter<bool>("query_device_info", true);
  // Block size for averaging decimation: the mean of every N parsed samples is
  // published on ~/data. N=1 publishes every sample unchanged; e.g. a 2000 Hz
  // device stream with N=20 yields ~100 Hz averaged samples (noise down ~sqrt N).
  publish_every_n_ = this->declare_parameter<int64_t>("publish_every_n", 1);
  if (publish_every_n_ < 1) {
    RCLCPP_WARN(
      get_logger(), "publish_every_n=%ld is invalid; clamping to 1.", publish_every_n_);
    publish_every_n_ = 1;
  }
  expected_rate_hz_ = this->declare_parameter<double>("expected_rate_hz", 100.0);

  // 9-element row-major covariance matrices. Yaw variance defaults large
  // because Calibrated HR heading is relative (no magnetometer).
  orientation_cov_ = this->declare_parameter<std::vector<double>>(
    "orientation_covariance",
    {0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 100.0});
  angular_velocity_cov_ = this->declare_parameter<std::vector<double>>(
    "angular_velocity_covariance",
    {4.0e-4, 0.0, 0.0, 0.0, 4.0e-4, 0.0, 0.0, 0.0, 4.0e-4});
  linear_acceleration_cov_ = this->declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance",
    {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04});
}

ImuDriverNode::CallbackReturn
ImuDriverNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(
    get_logger(), "Configuring IMU driver (port=%s, baud=%ld, format=%s)",
    port_.c_str(), baudrate_, output_format_.c_str());

  if (output_format_ != "calib_hr") {
    RCLCPP_ERROR(
      get_logger(), "Unsupported output_format '%s' (only 'calib_hr' is implemented).",
      output_format_.c_str());
    return CallbackReturn::FAILURE;
  }

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/data", rclcpp::SensorDataQoS());
  temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
    "~/temperature", rclcpp::SensorDataQoS());

  diag_ = std::make_unique<diagnostic_updater::Updater>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface());
  diag_->setHardwareID(port_);
  diag_->add("IMU", std::bind(&ImuDriverNode::produce_diagnostics, this, std::placeholders::_1));

  std::string err;
  int fd = open_serial(port_, static_cast<int>(baudrate_), &err);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial %s: %s", port_.c_str(), err.c_str());
    return CallbackReturn::FAILURE;
  }
  fd_ = fd;
  parser_.reset();

  // Capture device identity/health now, on an idle line, before any streaming
  // starts. Doing this here (rather than in on_activate, racing the CalibHR
  // stream) makes the GetDevInfo/GetBIT replies arrive cleanly. Identity is
  // non-critical, so a failure is logged but does not fail configuration.
  if (query_device_info_) {
    if (!query_device_identity()) {
      RCLCPP_WARN(get_logger(), "Device identity/BIT query incomplete; continuing.");
    }
  }
  return CallbackReturn::SUCCESS;
}

ImuDriverNode::CallbackReturn
ImuDriverNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  imu_pub_->on_activate();
  temp_pub_->on_activate();

  // Identity was captured in on_configure. Reset to a clean stream: Stop halts
  // any auto-start format; GetBIT requests the power-on health word (this
  // firmware only answers it in the streaming context, and its reply may be
  // delayed, so it is caught by the read loop below); CalibHR starts the data
  // stream. Residual pre-Stop frames are simply early CalibHR samples.
  stop_stream();
  if (query_device_info_) {
    send_command(Command::kGetBIT);
  }
  send_command(Command::kCalibHR);

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    last_diag_time_ = now();
    samples_at_last_diag_ = samples_total_;
  }

  // Start each activation with an empty block so a partial block left over from
  // a previous activation can't merge with fresh samples.
  block_ = SampleBlock{};
  running_ = true;
  io_thread_ = std::thread(&ImuDriverNode::read_loop, this);
  RCLCPP_INFO(get_logger(), "IMU driver activated.");
  return CallbackReturn::SUCCESS;
}

ImuDriverNode::CallbackReturn
ImuDriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  running_ = false;
  int fd = fd_.load();
  if (fd >= 0) {
    stop_stream();
    ::shutdown(fd, SHUT_RDWR);  // unblock the blocking read()
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  imu_pub_->on_deactivate();
  temp_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

ImuDriverNode::CallbackReturn
ImuDriverNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
  imu_pub_.reset();
  temp_pub_.reset();
  diag_.reset();
  return CallbackReturn::SUCCESS;
}

ImuDriverNode::CallbackReturn
ImuDriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  running_ = false;
  int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

bool ImuDriverNode::write_all(const std::vector<uint8_t> & bytes)
{
  int fd = fd_.load();
  if (fd < 0) {
    return false;
  }
  std::lock_guard<std::mutex> lk(write_mutex_);
  const uint8_t * p = bytes.data();
  size_t remaining = bytes.size();
  while (remaining > 0) {
    ssize_t n = ::write(fd, p, remaining);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      return false;
    }
    p += n;
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

void ImuDriverNode::send_command(Command code)
{
  if (!write_all(build_command(code))) {
    RCLCPP_WARN(get_logger(), "Failed to send command 0x%02X.", static_cast<int>(code));
  }
}

void ImuDriverNode::stop_stream()
{
  write_all(build_command(Command::kStop));
}

bool ImuDriverNode::query_device_identity()
{
  int fd = fd_.load();
  if (fd < 0) {
    return false;
  }
  // Halt any auto-start stream and clear buffered/in-transit data so the query
  // replies arrive on an otherwise-silent line.
  write_all(build_command(Command::kStop));
  drain_input(fd, 150);

  // GetDevInfo answers reliably on an idle line. (GetBIT does not on this
  // firmware -- it only responds in the streaming context, so it is issued in
  // on_activate and captured by the continuous read loop instead.) A drain
  // after the command lets the device finish emitting the reply.
  bool info_ok = false;
  for (int attempt = 0; attempt < 3 && !info_ok; ++attempt) {
    info_ok = wait_for_response(fd, Command::kGetDevInfo, kDataIdDevInfo, 300);
    drain_input(fd, 30);
  }

  // Leave the device idle; on_activate starts the CalibHR stream.
  drain_input(fd, 50);
  return info_ok;
}

bool ImuDriverNode::wait_for_response(int fd, Command cmd, uint8_t expected_id, int timeout_ms)
{
  FrameParser parser;
  if (!write_all(build_command(cmd))) {
    return false;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t buf[1024];
  std::vector<Frame> frames;
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) {
      break;
    }
    ssize_t n = read_with_timeout(fd, buf, sizeof(buf), static_cast<int>(remaining));
    if (n < 0) {
      return false;
    }
    if (n == 0) {
      continue;
    }
    frames.clear();
    parser.push(buf, static_cast<size_t>(n), frames);
    for (const auto & f : frames) {
      if (f.data_id != expected_id) {
        continue;
      }
      if (expected_id == kDataIdDevInfo) {
        DevInfo info;
        if (parse_dev_info(f.payload.data(), f.payload.size(), info)) {
          RCLCPP_INFO(
            get_logger(), "Device info: serial='%s' fw='%s' type=%u",
            info.serial.c_str(), info.firmware.c_str(), info.imu_type);
          std::lock_guard<std::mutex> lk(state_mutex_);
          dev_info_ = info;
          return true;
        }
      } else if (expected_id == kDataIdBit) {
        BitStatus s;
        if (parse_bit(f.payload.data(), f.payload.size(), s)) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          bit_status_ = s;
          return true;
        }
      }
    }
  }
  RCLCPP_WARN(
    get_logger(), "No response to command 0x%02X within %d ms.",
    static_cast<int>(cmd), timeout_ms);
  return false;
}

void ImuDriverNode::read_loop()
{
  uint8_t buf[1024];
  std::vector<Frame> frames;
  while (running_.load()) {
    int fd = fd_.load();
    if (fd < 0) {break;}
    ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n <= 0) {
      if (n < 0 && errno == EINTR) {continue;}
      if (running_.load()) {
        RCLCPP_WARN(get_logger(), "Serial read returned %zd (errno=%d).", n, errno);
      }
      break;
    }
    frames.clear();
    parser_.push(buf, static_cast<size_t>(n), frames);
    for (const auto & f : frames) {
      handle_frame(f);
    }
  }
}

void ImuDriverNode::handle_frame(const Frame & f)
{
  if (f.data_id == kDataIdCalibHr) {
    CalibHrData d;
    if (!parse_calib_hr(f.payload.data(), f.payload.size(), d, gravity_)) {
      return;
    }
    // Track every received sample for device-stream health (rate, USW, temp),
    // independent of decimation, so faults are caught at the full device rate.
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      last_sample_time_ = now();
      ++samples_total_;
      last_usw_ = d.usw;
      last_temperature_c_ = d.temperature_c;
    }
    // Average each block of publish_every_n samples and publish the mean, so a
    // fast device stream (e.g. 2 kHz) is throttled to an EKF-friendly rate while
    // still using every sample (lower noise + anti-alias vs. dropping N-1 of N).
    accumulate_sample(d);
    return;
  }
  if (f.data_id == kDataIdDevInfo) {
    DevInfo info;
    if (parse_dev_info(f.payload.data(), f.payload.size(), info)) {
      RCLCPP_INFO(
        get_logger(), "Device info: serial='%s' fw='%s' type=%u",
        info.serial.c_str(), info.firmware.c_str(), info.imu_type);
      std::lock_guard<std::mutex> lk(state_mutex_);
      dev_info_ = info;
    }
    return;
  }
  if (f.data_id == kDataIdBit) {
    BitStatus s;
    if (parse_bit(f.payload.data(), f.payload.size(), s)) {
      std::lock_guard<std::mutex> lk(state_mutex_);
      bit_status_ = s;
    }
    return;
  }
  std::lock_guard<std::mutex> lk(state_mutex_);
  ++unknown_frames_;
  const bool first_seen = unknown_ids_.find(f.data_id) == unknown_ids_.end();
  ++unknown_ids_[f.data_id];
  if (first_seen) {
    RCLCPP_INFO(
      get_logger(),
      "Unhandled frame data_id=0x%02X (msg_type=%u, payload=%zu bytes); ignoring.",
      f.data_id, f.msg_type, f.payload.size());
  }
}

void ImuDriverNode::accumulate_sample(const CalibHrData & d)
{
  for (int i = 0; i < 3; ++i) {
    block_.angular_velocity[i] += d.angular_velocity[i];
    block_.linear_acceleration[i] += d.linear_acceleration[i];
  }
  block_.temperature_c += d.temperature_c;
  block_.usw_raw |= d.usw.raw;  // keep every fault bit seen within the block
  block_.last = d;              // latest orientation/counter for the block stamp
  ++block_.count;

  if (block_.count >= static_cast<uint64_t>(publish_every_n_)) {
    publish_block();
  }
}

void ImuDriverNode::publish_block()
{
  if (block_.count == 0) {
    return;
  }
  // Start from the most recent sample so orientation (which cannot be linearly
  // averaged) and the counter reflect the block's latest state, then overwrite
  // the linearly-averageable channels with the block mean.
  CalibHrData avg = block_.last;
  const double inv = 1.0 / static_cast<double>(block_.count);
  for (int i = 0; i < 3; ++i) {
    avg.angular_velocity[i] = block_.angular_velocity[i] * inv;
    avg.linear_acceleration[i] = block_.linear_acceleration[i] * inv;
  }
  avg.temperature_c = block_.temperature_c * inv;
  avg.usw = decode_usw(block_.usw_raw);

  publish_imu(avg);
  block_ = SampleBlock{};
}

void ImuDriverNode::publish_imu(const CalibHrData & d)
{
  const rclcpp::Time stamp = now();

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;

  if (publish_orientation_) {
    msg.orientation.w = d.orientation.w;
    msg.orientation.x = d.orientation.x;
    msg.orientation.y = d.orientation.y;
    msg.orientation.z = d.orientation.z;
    fill_cov(msg.orientation_covariance, orientation_cov_);
  } else {
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // "no orientation" per sensor_msgs/Imu.
  }

  msg.angular_velocity.x = d.angular_velocity[0];
  msg.angular_velocity.y = d.angular_velocity[1];
  msg.angular_velocity.z = d.angular_velocity[2];
  fill_cov(msg.angular_velocity_covariance, angular_velocity_cov_);

  msg.linear_acceleration.x = d.linear_acceleration[0];
  msg.linear_acceleration.y = d.linear_acceleration[1];
  msg.linear_acceleration.z = d.linear_acceleration[2];
  fill_cov(msg.linear_acceleration_covariance, linear_acceleration_cov_);

  imu_pub_->publish(msg);

  if (publish_temperature_) {
    sensor_msgs::msg::Temperature t;
    t.header.stamp = stamp;
    t.header.frame_id = frame_id_;
    t.temperature = d.temperature_c;
    t.variance = 0.0;
    temp_pub_->publish(t);
  }

  std::lock_guard<std::mutex> lk(state_mutex_);
  ++published_total_;
}

void ImuDriverNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lk(state_mutex_);

  const rclcpp::Time tnow = now();
  double rate = 0.0;
  double publish_rate = 0.0;
  if (last_diag_time_.nanoseconds() > 0) {
    const double dt = (tnow - last_diag_time_).seconds();
    if (dt > 0.0) {
      rate = static_cast<double>(samples_total_ - samples_at_last_diag_) / dt;
      publish_rate = static_cast<double>(published_total_ - published_at_last_diag_) / dt;
    }
  }
  last_diag_time_ = tnow;
  samples_at_last_diag_ = samples_total_;
  published_at_last_diag_ = published_total_;

  const bool have_data = last_sample_time_.nanoseconds() > 0;
  const double age = have_data ? (tnow - last_sample_time_).seconds() : 1e9;

  const bool sensor_fault = last_usw_ &&
    (last_usw_->sensors_comm_failure || last_usw_->sensors_config_failure);
  const bool range_warn = last_usw_ &&
    (last_usw_->ang_rate_x_out_of_range || last_usw_->ang_rate_y_out_of_range ||
    last_usw_->ang_rate_z_out_of_range || last_usw_->temperature_out_of_range);

  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!have_data || age > 1.0) {
    stat.summary(DiagnosticStatus::ERROR, "No IMU samples");
  } else if (sensor_fault) {
    stat.summary(DiagnosticStatus::ERROR, "Sensor fault (USW)");
  } else if (rate < 0.5 * expected_rate_hz_) {
    stat.summary(DiagnosticStatus::WARN, "Sample rate below expected");
  } else if (range_warn) {
    stat.summary(DiagnosticStatus::WARN, "Range/temperature warning (USW)");
  } else {
    stat.summary(DiagnosticStatus::OK, "OK");
  }

  stat.add("sample_rate_hz", rate);
  stat.add("expected_rate_hz", expected_rate_hz_);
  stat.add("publish_rate_hz", publish_rate);
  stat.add("publish_every_n", static_cast<int>(publish_every_n_));
  stat.add("samples_total", static_cast<int>(samples_total_));
  stat.add("published_total", static_cast<int>(published_total_));
  stat.add("data_age_s", have_data ? age : -1.0);
  stat.add("frames_parsed", static_cast<int>(parser_.frames_parsed()));
  stat.add("checksum_errors", static_cast<int>(parser_.checksum_errors()));
  stat.add("bytes_discarded", static_cast<int>(parser_.bytes_discarded()));
  stat.add("unknown_frames", static_cast<int>(unknown_frames_));
  if (!unknown_ids_.empty()) {
    std::string ids;
    char buf[16];
    for (const auto & kv : unknown_ids_) {
      std::snprintf(buf, sizeof(buf), "0x%02X:%zu", kv.first, kv.second);
      if (!ids.empty()) {ids += " ";}
      ids += buf;
    }
    stat.add("unknown_ids", ids);
  }
  if (last_temperature_c_) {
    stat.add("temperature_c", *last_temperature_c_);
  }
  if (last_usw_) {
    stat.addf("usw", "0x%04X", last_usw_->raw);
  }
  if (dev_info_) {
    stat.add("serial", dev_info_->serial);
    stat.add("firmware", dev_info_->firmware);
    stat.add("imu_type", static_cast<int>(dev_info_->imu_type));
  }
  if (bit_status_) {
    stat.add("bit_accel_ok", bit_status_->accel_ok);
    stat.add("bit_gyro_ok", bit_status_->gyro_ok);
    stat.add("bit_flash_ok", bit_status_->flash_ok);
  }
}

}  // namespace imu_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<imu_driver::ImuDriverNode>(rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
