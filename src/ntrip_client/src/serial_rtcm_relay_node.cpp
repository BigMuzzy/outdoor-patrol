// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// Serial-radio RTCM relay. Opens a UART (typically a 900 MHz / LoRa /
// SiK radio modem), splits the incoming RTCM3 byte stream into validated
// frames using `RtcmFramer`, and republishes each frame on `rtcm/out`.
//
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"

#include "ntrip_client/rtcm_framer.hpp"
#include "ntrip_client/serial_port.hpp"

namespace ntrip_client
{

class SerialRtcmRelayNode : public rclcpp::Node
{
public:
  explicit SerialRtcmRelayNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("serial_rtcm_relay", options),
    framer_(
      [this](const std::vector<uint8_t> & frame) {publish_frame(frame);})
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB1");
    baudrate_ = static_cast<int>(declare_parameter<int64_t>("baudrate", 57600));
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");
    reopen_backoff_s_ = declare_parameter<double>("reopen_backoff_s", 2.0);

    rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>("rtcm/out", rclcpp::QoS(50));

    running_ = true;
    io_thread_ = std::thread([this] {run();});
  }

  ~SerialRtcmRelayNode() override
  {
    running_ = false;
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
      ::close(fd);  // unblocks read()
    }
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  void publish_frame(const std::vector<uint8_t> & frame)
  {
    rtcm_msgs::msg::Message msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.message.assign(frame.begin(), frame.end());
    rtcm_pub_->publish(msg);
  }

  void interruptible_sleep(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    while (running_ && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void run()
  {
    uint8_t buf[1024];
    while (running_) {
      std::string err;
      const int fd = open_serial(port_, baudrate_, &err);
      if (fd < 0) {
        RCLCPP_WARN(get_logger(), "Open %s @ %d failed (%s); retry in %.1fs",
          port_.c_str(), baudrate_, err.c_str(), reopen_backoff_s_);
        interruptible_sleep(reopen_backoff_s_);
        continue;
      }
      RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baudrate_);
      fd_ = fd;
      framer_.reset();

      while (running_) {
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
          framer_.push(buf, static_cast<std::size_t>(n));
        } else if (n == 0) {
          break;  // EOF / hot-unplug
        } else {
          if (errno == EINTR) {continue;}
          if (!running_) {break;}
          RCLCPP_WARN(get_logger(), "read error: %s", std::strerror(errno));
          break;
        }
      }

      const int closing = fd_.exchange(-1);
      if (closing >= 0) {
        ::close(closing);
      }
      if (running_) {
        interruptible_sleep(reopen_backoff_s_);
      }
    }
  }

  std::string port_;
  int baudrate_{57600};
  std::string frame_id_;
  double reopen_backoff_s_{2.0};

  RtcmFramer framer_;
  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
  std::atomic<bool> running_{false};
  std::atomic<int> fd_{-1};
  std::thread io_thread_;
};

}  // namespace ntrip_client

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ntrip_client::SerialRtcmRelayNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
