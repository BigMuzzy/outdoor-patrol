// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/um982_driver_node.hpp"

#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "um982_driver/command_builder.hpp"
#include "um982_driver/serial_port.hpp"

namespace um982_driver
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

// A GST sentence's measured sigmas are used for NavSatFix covariance only
// while fresh; beyond this age we revert to the quality/HDOP heuristic.
constexpr double kGstStaleSec = 2.0;

// Heuristic horizontal accuracy multipliers keyed on GGA fix quality.
// Used to seed NavSatFix covariance when the receiver does not provide
// a BESTNAV/BESTPOSA sigma directly. Units: metres of 1-sigma per HDOP.
double quality_to_sigma_m(NmeaFixQuality q)
{
  switch (q) {
    case NmeaFixQuality::kRtkFix: return 0.02;
    case NmeaFixQuality::kRtkFloat: return 0.30;
    case NmeaFixQuality::kDgps: return 1.0;
    case NmeaFixQuality::kSps: return 3.0;
    case NmeaFixQuality::kPps: return 3.0;
    case NmeaFixQuality::kDeadReckoning: return 10.0;
    default: return 100.0;
  }
}
}  // namespace

Um982DriverNode::Um982DriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("um982_driver", options),
  last_fix_time_(0, 0, RCL_ROS_TIME),
  last_rtcm_in_time_(0, 0, RCL_ROS_TIME),
  last_gst_time_(0, 0, RCL_ROS_TIME)
{
  declare_parameters();
}

Um982DriverNode::~Um982DriverNode()
{
  running_ = false;
  int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void Um982DriverNode::declare_parameters()
{
  port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = this->declare_parameter<int64_t>("baudrate", 115200);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "gnss_link");
  heading_frame_id_ = this->declare_parameter<std::string>("heading_frame_id", "gnss_link");
  // Drop /heading when the receiver's KSXT heading-quality is below this
  // (0=invalid, 1=single, 2=RTK float, 3=RTK fixed). Default 1 rejects only
  // the invalid 0.00 deg placeholder emitted when ANT2 has no signal.
  min_heading_quality_ = static_cast<int>(
    this->declare_parameter<int64_t>("min_heading_quality", 1));
  mode_ = this->declare_parameter<std::string>("mode", "rover");
  rover_dynamics_ = this->declare_parameter<std::string>("rover.dynamics", "AUTOMOTIVE");
  base_lat_ = this->declare_parameter<double>("base_fixed.lat", 0.0);
  base_lon_ = this->declare_parameter<double>("base_fixed.lon", 0.0);
  base_height_ = this->declare_parameter<double>("base_fixed.height", 0.0);
  survey_seconds_ = this->declare_parameter<double>("base_survey.averaging_seconds", 60.0);
  survey_dist_m_ = this->declare_parameter<double>("base_survey.max_position_std_m", 0.0);
  heading2_mode_ = this->declare_parameter<std::string>("heading2.mode", "");
  rtcm_ids_ = this->declare_parameter<std::vector<int64_t>>(
    "rtcm_messages.ids", std::vector<int64_t>{1006, 1033, 1074, 1084, 1094, 1124});
  rtcm_period_s_ = this->declare_parameter<double>("rtcm_messages.period_s", 1.0);
  rtcm_out_com_ = this->declare_parameter<std::string>("rtcm_messages.com", "com2");
  output_messages_ = this->declare_parameter<std::vector<std::string>>(
    "output_messages.names", std::vector<std::string>{"GPGGA", "GPGST", "GPRMC", "GPVTG", "KSXT"});
  output_period_s_ = this->declare_parameter<double>("output_messages.period_s", 0.2);
  output_com_ = this->declare_parameter<std::string>("output_messages.com", "");
  antenna_h_ = this->declare_parameter<double>("antenna_offset.h", 0.0);
  antenna_e_ = this->declare_parameter<double>("antenna_offset.e", 0.0);
  antenna_n_ = this->declare_parameter<double>("antenna_offset.n", 0.0);
  save_config_ = this->declare_parameter<bool>("save_config_on_configure", false);
  unlogall_on_configure_ = this->declare_parameter<bool>("unlogall_on_configure", true);
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

  diag_ = std::make_unique<diagnostic_updater::Updater>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface());
  diag_->setHardwareID(port_);
  diag_->add(
    "GNSS",
    std::bind(&Um982DriverNode::produce_diagnostics, this, std::placeholders::_1));

  std::string err;
  int fd = open_serial(port_, static_cast<int>(baudrate_), &err);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial %s: %s", port_.c_str(), err.c_str());
    return CallbackReturn::FAILURE;
  }
  fd_ = fd;
  splitter_.reset();
  send_init_commands();
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

  running_ = true;
  io_thread_ = std::thread(&Um982DriverNode::read_loop, this);
  RCLCPP_INFO(get_logger(), "UM982 driver activated.");
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  running_ = false;
  int fd = fd_.load();
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);  // unblock blocking read()
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
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
  int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
  rtcm_sub_.reset();
  fix_pub_.reset();
  vel_pub_.reset();
  heading_pub_.reset();
  nmea_pub_.reset();
  time_pub_.reset();
  diag_.reset();
  return CallbackReturn::SUCCESS;
}

Um982DriverNode::CallbackReturn
Um982DriverNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
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

void Um982DriverNode::on_rtcm(const rtcm_msgs::msg::Message::SharedPtr msg)
{
  if (msg->message.empty()) {
    return;
  }
  if (!write_all(std::string(msg->message.begin(), msg->message.end()))) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to write %zu RTCM bytes to serial.", msg->message.size());
    return;
  }
  std::lock_guard<std::mutex> lk(state_mutex_);
  rtcm_bytes_in_ += msg->message.size();
  last_rtcm_in_time_ = now();
}

bool Um982DriverNode::write_all(const std::string & cmd)
{
  int fd = fd_.load();
  if (fd < 0) {
    return false;
  }
  std::lock_guard<std::mutex> lk(write_mutex_);
  const char * p = cmd.data();
  size_t remaining = cmd.size();
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

void Um982DriverNode::send_init_commands()
{
  if (unlogall_on_configure_) {
    write_all(build_unlogall());
  }

  if (antenna_h_ != 0.0 || antenna_e_ != 0.0 || antenna_n_ != 0.0) {
    write_all(build_antenna_delta_hen(antenna_h_, antenna_e_, antenna_n_));
  }

  if (mode_ == "rover") {
    write_all(build_mode_rover(rover_dynamics_));
  } else if (mode_ == "base_fixed") {
    write_all(build_mode_base_fixed(base_lat_, base_lon_, base_height_));
    for (auto id : rtcm_ids_) {
      write_all(build_rtcm_output(static_cast<int>(id), rtcm_out_com_, rtcm_period_s_));
    }
  } else if (mode_ == "base_survey") {
    write_all(build_mode_base_survey(survey_seconds_, survey_dist_m_));
    for (auto id : rtcm_ids_) {
      write_all(build_rtcm_output(static_cast<int>(id), rtcm_out_com_, rtcm_period_s_));
    }
  } else if (mode_ == "heading2") {
    write_all(build_mode_heading2(heading2_mode_));
  } else {
    RCLCPP_WARN(get_logger(), "Unknown mode '%s' — leaving receiver as-is.", mode_.c_str());
  }

  for (const auto & m : output_messages_) {
    write_all(build_log(m, output_com_, output_period_s_));
  }

  if (save_config_) {
    write_all(build_saveconfig());
    RCLCPP_INFO(get_logger(), "saveconfig issued (NVM written).");
  }
}

void Um982DriverNode::read_loop()
{
  uint8_t buf[1024];
  std::vector<Sentence> sentences;
  while (running_.load()) {
    int fd = fd_.load();
    if (fd < 0) {break;}
    ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n <= 0) {
      if (n < 0 && errno == EINTR) {continue;}
      RCLCPP_WARN(get_logger(), "Serial read returned %zd (errno=%d).", n, errno);
      break;
    }
    sentences.clear();
    splitter_.push(buf, static_cast<size_t>(n), sentences);
    for (const auto & s : sentences) {
      handle_sentence(s);
    }
  }
}

void Um982DriverNode::handle_sentence(const Sentence & s)
{
  // The UM982 emits two ASCII families that use *different* checksums:
  //   - NMEA '$...' sentences (incl. proprietary $KSXT) end with an 8-bit XOR
  //     checksum ('*HH').
  //   - Unicore '#...A' messages (e.g. #UNIHEADINGA, #VERSIONA) end with a
  //     32-bit CRC ('*HHHHHHHH'), per the Reference Commands Manual Appendix 1.
  // Each must be verified with the matching scheme; otherwise every valid
  // Unicore message is miscounted as an NMEA checksum failure (and dropped).
  if (s.kind == SentenceKind::kUnicoreAscii) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (verify_unicore_checksum(s.text)) {
      ++unicore_in_;
    } else {
      ++bad_checksums_;
    }
    return;  // Payload not consumed yet — heading/position come from $KSXT.
  }

  if (!verify_nmea_checksum(s.text)) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    ++bad_checksums_;
    return;
  }

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    ++sentences_in_;
  }

  // Publish raw NMEA so NTRIP client can latch GGA.
  if (nmea_pub_ && nmea_pub_->is_activated()) {
    nmea_msgs::msg::Sentence raw;
    raw.header.stamp = now();
    raw.header.frame_id = frame_id_;
    raw.sentence = s.text;
    nmea_pub_->publish(raw);
  }

  // Identifier is fields[0]; cheap substring match avoids re-splitting.
  // GGA / RMC / VTG / KSXT.
  if (s.text.size() > 6 && s.text.compare(3, 3, "GGA") == 0) {
    if (auto g = parse_gga(s.text)) {
      publish_gga(*g, s.text);
    }
  } else if (s.text.size() > 6 && s.text.compare(3, 3, "RMC") == 0) {
    if (auto r = parse_rmc(s.text)) {
      publish_rmc(*r);
    }
  } else if (s.text.size() > 6 && s.text.compare(3, 3, "VTG") == 0) {
    if (auto v = parse_vtg(s.text)) {
      publish_vtg(*v);
    }
  } else if (s.text.size() > 6 && s.text.compare(3, 3, "GST") == 0) {
    if (auto g = parse_gst(s.text)) {
      std::lock_guard<std::mutex> lk(state_mutex_);
      last_gst_ = *g;
      last_gst_time_ = now();
    }
  } else if (s.text.size() > 5 && s.text.compare(1, 4, "KSXT") == 0) {
    if (auto k = parse_ksxt(s.text)) {
      publish_ksxt(*k);
    }
  }
}

void Um982DriverNode::publish_gga(const NmeaGga & gga, const std::string & /*raw*/)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = now();
  fix.header.frame_id = frame_id_;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  switch (gga.quality) {
    case NmeaFixQuality::kInvalid:
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case NmeaFixQuality::kSps:
    case NmeaFixQuality::kPps:
    case NmeaFixQuality::kDeadReckoning:
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case NmeaFixQuality::kDgps:
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case NmeaFixQuality::kRtkFix:
    case NmeaFixQuality::kRtkFloat:
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
  }
  fix.latitude = gga.latitude_deg;
  fix.longitude = gga.longitude_deg;
  fix.altitude = gga.altitude_m;
  // Prefer the receiver's measured 1-sigma errors from a recent GST sentence;
  // fall back to a quality/HDOP heuristic when GST is unavailable or stale.
  std::optional<NmeaGst> gst;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (last_gst_ &&
      (now() - last_gst_time_).seconds() < kGstStaleSec)
    {
      gst = last_gst_;
    }
  }
  if (gst) {
    // GST reports lat (North), lon (East) and alt (Up) standard deviations.
    fix.position_covariance[0] = gst->std_lon_m * gst->std_lon_m;  // East
    fix.position_covariance[4] = gst->std_lat_m * gst->std_lat_m;  // North
    fix.position_covariance[8] = gst->std_alt_m * gst->std_alt_m;  // Up
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    double sigma = quality_to_sigma_m(gga.quality) * std::max(0.5, gga.hdop);
    double var = sigma * sigma;
    fix.position_covariance[0] = var;
    fix.position_covariance[4] = var;
    fix.position_covariance[8] = (var * 4.0);  // vertical is roughly 2x worse
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  }

  if (fix_pub_ && fix_pub_->is_activated()) {
    fix_pub_->publish(fix);
  }

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    last_quality_ = gga.quality;
    last_num_sats_ = gga.num_satellites;
    last_hdop_ = gga.hdop;
    last_correction_age_s_ = gga.age_of_corrections_s;
    last_fix_time_ = fix.header.stamp;
  }
}

void Um982DriverNode::publish_rmc(const NmeaRmc & rmc)
{
  if (!rmc.valid) {return;}
  // Use RMC primarily for TimeReference.
  sensor_msgs::msg::TimeReference tref;
  tref.header.stamp = now();
  tref.header.frame_id = frame_id_;
  tref.source = "GNSS";
  // Best-effort UTC: rmc.utc is HHMMSS.SS, date is DDMMYY. Compose if both.
  if (rmc.utc.size() >= 6 && rmc.date_ddmmyy.size() == 6) {
    int hh = std::stoi(rmc.utc.substr(0, 2));
    int mm = std::stoi(rmc.utc.substr(2, 2));
    double ss = std::stod(rmc.utc.substr(4));
    int dd = std::stoi(rmc.date_ddmmyy.substr(0, 2));
    int mo = std::stoi(rmc.date_ddmmyy.substr(2, 2));
    int yy = std::stoi(rmc.date_ddmmyy.substr(4, 2)) + 2000;
    struct tm t{};
    t.tm_year = yy - 1900;
    t.tm_mon = mo - 1;
    t.tm_mday = dd;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = static_cast<int>(ss);
    time_t epoch = timegm(&t);
    tref.time_ref = rclcpp::Time(static_cast<int64_t>(epoch), 0, RCL_ROS_TIME);
  }
  if (time_pub_ && time_pub_->is_activated()) {
    time_pub_->publish(tref);
  }
}

void Um982DriverNode::publish_vtg(const NmeaVtg & vtg)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header.stamp = now();
  twist.header.frame_id = frame_id_;
  if (vtg.course_over_ground_true_deg.has_value()) {
    double cog_rad = *vtg.course_over_ground_true_deg * kDegToRad;
    twist.twist.twist.linear.x = vtg.speed_over_ground_mps * std::cos(cog_rad);
    twist.twist.twist.linear.y = vtg.speed_over_ground_mps * std::sin(cog_rad);
  } else {
    twist.twist.twist.linear.x = vtg.speed_over_ground_mps;
  }
  // Coarse covariance: 0.5 m/s 1-sigma on each horizontal axis.
  twist.twist.covariance[0] = 0.25;
  twist.twist.covariance[7] = 0.25;
  twist.twist.covariance[14] = 1.0;
  if (vel_pub_ && vel_pub_->is_activated()) {
    vel_pub_->publish(twist);
  }
}

void Um982DriverNode::publish_ksxt(const KsxtSentence & k)
{
  if (!k.heading_deg.has_value()) {return;}
  // Gate on the receiver's dual-antenna heading-solution quality (KSXT:
  // 0=invalid, 1=single, 2=RTK float, 3=RTK fixed). Quality 0 means the
  // ANT1->ANT2 baseline is NOT solved (e.g. the secondary antenna ANT2 has no
  // signal); the receiver then emits a placeholder 0.00 deg. Publishing that
  // would feed a confidently-wrong absolute yaw into the EKF, so drop it and
  // let localization fall back to wheel-odometry yaw.
  if (static_cast<int>(k.heading_quality) < min_heading_quality_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Dropping GNSS heading: KSXT heading-quality %u < %d "
      "(dual-antenna baseline unsolved; check secondary antenna ANT2).",
      static_cast<unsigned>(k.heading_quality), min_heading_quality_);
    return;
  }
  // Convert heading (deg, CW from True North) to a quaternion expressing
  // the rotation about Z (ENU yaw, CCW from East). yaw = pi/2 - heading.
  double heading_rad = *k.heading_deg * kDegToRad;
  double yaw = M_PI / 2.0 - heading_rad;
  geometry_msgs::msg::QuaternionStamped q;
  q.header.stamp = now();
  q.header.frame_id = heading_frame_id_;
  q.quaternion.x = 0.0;
  q.quaternion.y = 0.0;
  q.quaternion.z = std::sin(yaw / 2.0);
  q.quaternion.w = std::cos(yaw / 2.0);
  if (heading_pub_ && heading_pub_->is_activated()) {
    heading_pub_->publish(q);
  }
}

void Um982DriverNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";
  switch (last_quality_) {
    case NmeaFixQuality::kRtkFix:
      msg = "RTK Fix"; break;
    case NmeaFixQuality::kRtkFloat:
      msg = "RTK Float";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case NmeaFixQuality::kDgps:
      msg = "DGPS";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case NmeaFixQuality::kSps:
    case NmeaFixQuality::kPps:
      msg = "SPS (no corrections)";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case NmeaFixQuality::kInvalid:
      msg = "No fix";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
    default:
      msg = "Other";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
  }
  stat.summary(level, msg);
  stat.add("fix_quality", static_cast<int>(last_quality_));
  stat.add("num_satellites", static_cast<int>(last_num_sats_));
  stat.add("hdop", last_hdop_);
  if (last_correction_age_s_.has_value()) {
    stat.add("correction_age_s", *last_correction_age_s_);
  }
  if (last_gst_.has_value()) {
    stat.add("std_lat_m", last_gst_->std_lat_m);
    stat.add("std_lon_m", last_gst_->std_lon_m);
    stat.add("std_alt_m", last_gst_->std_alt_m);
  }
  stat.add("rtcm_bytes_in", static_cast<int>(rtcm_bytes_in_));
  stat.add("sentences_in", static_cast<int>(sentences_in_));
  stat.add("unicore_msgs_in", static_cast<int>(unicore_in_));
  stat.add("bad_checksums", static_cast<int>(bad_checksums_));
  stat.add("splitter_overflow", static_cast<int>(splitter_.overflow_count()));
  stat.add("splitter_bytes_discarded", static_cast<int>(splitter_.bytes_discarded()));
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
