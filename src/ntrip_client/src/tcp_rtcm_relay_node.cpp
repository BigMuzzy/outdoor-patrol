// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// TCP RTCM relay. Reads a raw RTCM3 byte stream from a TCP peer, splits
// it into validated frames using `RtcmFramer` and republishes each frame
// on `rtcm/out` as `rtcm_msgs/Message`.
//
// Two roles:
//   role=client : connect to `host:port`, reconnect with exponential backoff
//   role=server : listen on `bind_address:port`, accept one peer at a time
//
#include <sys/socket.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"

#include "ntrip_client/posix_io.hpp"
#include "ntrip_client/rtcm_framer.hpp"

namespace ntrip_client
{

class TcpRtcmRelayNode : public rclcpp::Node
{
public:
  explicit TcpRtcmRelayNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tcp_rtcm_relay", options),
    framer_(
      [this](const std::vector<uint8_t> & frame) {publish_frame(frame);})
  {
    role_ = declare_parameter<std::string>("role", "client");  // client|server
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    port_ = static_cast<int>(declare_parameter<int64_t>("port", 2102));
    reconnect_min_s_ = declare_parameter<double>("reconnect_backoff_s_min", 1.0);
    reconnect_max_s_ = declare_parameter<double>("reconnect_backoff_s_max", 30.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");

    rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>("rtcm/out", rclcpp::QoS(50));

    running_ = true;
    if (role_ == "server") {
      io_thread_ = std::thread([this] {run_server();});
    } else {
      if (role_ != "client") {
        RCLCPP_WARN(get_logger(), "Unknown role '%s', defaulting to client.", role_.c_str());
        role_ = "client";
      }
      io_thread_ = std::thread([this] {run_client();});
    }
  }

  ~TcpRtcmRelayNode() override
  {
    running_ = false;
    // Force any blocking accept/recv to return.
    const int lfd = listen_fd_.exchange(-1);
    if (lfd >= 0) {
      ::shutdown(lfd, SHUT_RDWR);
      ::close(lfd);
    }
    const int cfd = conn_fd_.exchange(-1);
    if (cfd >= 0) {
      ::shutdown(cfd, SHUT_RDWR);
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

  /// Sleep up to `seconds`, returning early if `running_` clears.
  void interruptible_sleep(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    while (running_ && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void read_loop(int fd)
  {
    uint8_t buf[4096];
    while (running_) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n > 0) {
        framer_.push(buf, static_cast<std::size_t>(n));
      } else if (n == 0) {
        RCLCPP_INFO(get_logger(), "Peer closed connection.");
        break;
      } else {
        if (errno == EINTR) {continue;}
        RCLCPP_WARN(get_logger(), "recv error: %s", std::strerror(errno));
        break;
      }
    }
  }

  void run_client()
  {
    double backoff = reconnect_min_s_;
    while (running_) {
      std::string err;
      RCLCPP_INFO(get_logger(), "Connecting to %s:%d ...", host_.c_str(), port_);
      const int fd = tcp_connect(host_, port_, &err);
      if (fd < 0) {
        RCLCPP_WARN(get_logger(), "Connect failed (%s); retry in %.1fs", err.c_str(), backoff);
        interruptible_sleep(backoff);
        backoff = std::min(backoff * 2.0, reconnect_max_s_);
        continue;
      }
      RCLCPP_INFO(get_logger(), "Connected to %s:%d", host_.c_str(), port_);
      conn_fd_ = fd;
      backoff = reconnect_min_s_;
      framer_.reset();
      read_loop(fd);
      const int closing = conn_fd_.exchange(-1);
      if (closing >= 0) {
        ::close(closing);
      }
      if (running_) {
        interruptible_sleep(reconnect_min_s_);
      }
    }
  }

  void run_server()
  {
    while (running_) {
      std::string err;
      RCLCPP_INFO(get_logger(), "Listening on %s:%d ...", bind_address_.c_str(), port_);
      const int lfd = tcp_listen(bind_address_, port_, &err);
      if (lfd < 0) {
        RCLCPP_WARN(get_logger(), "Listen failed (%s); retry in %.1fs", err.c_str(),
          reconnect_max_s_);
        interruptible_sleep(reconnect_max_s_);
        continue;
      }
      listen_fd_ = lfd;

      while (running_) {
        const int cfd = ::accept(lfd, nullptr, nullptr);
        if (cfd < 0) {
          if (!running_) {break;}
          if (errno == EINTR) {continue;}
          RCLCPP_WARN(get_logger(), "accept error: %s", std::strerror(errno));
          break;
        }
        RCLCPP_INFO(get_logger(), "Peer connected.");
        conn_fd_ = cfd;
        framer_.reset();
        read_loop(cfd);
        const int closing = conn_fd_.exchange(-1);
        if (closing >= 0) {
          ::close(closing);
        }
      }

      const int closing_l = listen_fd_.exchange(-1);
      if (closing_l >= 0) {
        ::close(closing_l);
      }
    }
  }

  // Parameters
  std::string role_;
  std::string host_;
  std::string bind_address_;
  int port_{2102};
  double reconnect_min_s_{1.0};
  double reconnect_max_s_{30.0};
  std::string frame_id_;

  // Runtime
  RtcmFramer framer_;
  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
  std::atomic<bool> running_{false};
  std::atomic<int> conn_fd_{-1};
  std::atomic<int> listen_fd_{-1};
  std::thread io_thread_;
};

}  // namespace ntrip_client

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ntrip_client::TcpRtcmRelayNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
