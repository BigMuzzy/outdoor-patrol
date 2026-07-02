// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// NTRIP v1 / v2 caster client.
//
// Protocol summary
// ----------------
// NTRIP v1 (RFC-less; ICY response):
//
//   GET /MOUNTPOINT HTTP/1.0\r\n
//   User-Agent: NTRIP <ua>\r\n
//   Authorization: Basic <base64(user:pass)>\r\n
//   \r\n
//
//   Response on success starts with "ICY 200 OK\r\n\r\n" followed by a
//   raw RTCM3 byte stream. A failure starts with "SOURCETABLE 200 OK".
//
// NTRIP v2 (RFC 2616-style):
//
//   GET /MOUNTPOINT HTTP/1.1\r\n
//   Host: <host>:<port>\r\n
//   Ntrip-Version: Ntrip/2.0\r\n
//   User-Agent: NTRIP <ua>\r\n
//   Authorization: Basic <base64(user:pass)>\r\n
//   Connection: close\r\n
//   \r\n
//
//   Response: "HTTP/1.1 200 OK" + headers + blank line + body. The body
//   may be raw RTCM3 or chunked (Transfer-Encoding: chunked).
//
// GGA upload (VRS / nearest-base casters): when enabled, the latest
// `$GNGGA`/`$GPGGA` sentence received from `nmea_sentence` is written to
// the same socket every `gga_period_s`.
//
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rtcm_msgs/msg/message.hpp"

#include "ntrip_client/posix_io.hpp"
#include "ntrip_client/rtcm_framer.hpp"

namespace ntrip_client
{

namespace
{

constexpr char kBase64Alpha[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::string & in)
{
  std::string out;
  out.reserve(((in.size() + 2) / 3) * 4);
  std::size_t i = 0;
  while (i + 3 <= in.size()) {
    const uint32_t v = (static_cast<uint32_t>(static_cast<uint8_t>(in[i])) << 16) |
      (static_cast<uint32_t>(static_cast<uint8_t>(in[i + 1])) << 8) |
      static_cast<uint32_t>(static_cast<uint8_t>(in[i + 2]));
    out.push_back(kBase64Alpha[(v >> 18) & 0x3F]);
    out.push_back(kBase64Alpha[(v >> 12) & 0x3F]);
    out.push_back(kBase64Alpha[(v >> 6) & 0x3F]);
    out.push_back(kBase64Alpha[v & 0x3F]);
    i += 3;
  }
  const std::size_t rem = in.size() - i;
  if (rem == 1) {
    const uint32_t v = static_cast<uint32_t>(static_cast<uint8_t>(in[i])) << 16;
    out.push_back(kBase64Alpha[(v >> 18) & 0x3F]);
    out.push_back(kBase64Alpha[(v >> 12) & 0x3F]);
    out.push_back('=');
    out.push_back('=');
  } else if (rem == 2) {
    const uint32_t v = (static_cast<uint32_t>(static_cast<uint8_t>(in[i])) << 16) |
      (static_cast<uint32_t>(static_cast<uint8_t>(in[i + 1])) << 8);
    out.push_back(kBase64Alpha[(v >> 18) & 0x3F]);
    out.push_back(kBase64Alpha[(v >> 12) & 0x3F]);
    out.push_back(kBase64Alpha[(v >> 6) & 0x3F]);
    out.push_back('=');
  }
  return out;
}

bool write_all(int fd, const void * data, std::size_t len)
{
  const uint8_t * p = static_cast<const uint8_t *>(data);
  std::size_t left = len;
  while (left > 0) {
    const ssize_t n = ::send(fd, p, left, MSG_NOSIGNAL);
    if (n > 0) {
      p += n;
      left -= static_cast<std::size_t>(n);
    } else if (n < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

}  // namespace

class NtripClientNode : public rclcpp::Node
{
public:
  explicit NtripClientNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ntrip_client", options),
    framer_(
      [this](const std::vector<uint8_t> & frame) {publish_frame(frame);})
  {
    host_ = declare_parameter<std::string>("host", "");
    port_ = static_cast<int>(declare_parameter<int64_t>("port", 2101));
    mountpoint_ = declare_parameter<std::string>("mountpoint", "");
    username_ = declare_parameter<std::string>("username", "");
    password_ = declare_parameter<std::string>("password", "");
    ntrip_version_ = declare_parameter<std::string>("ntrip_version", "auto");
    user_agent_ = declare_parameter<std::string>("user_agent", "ntrip_client/0.0.1");
    send_gga_ = declare_parameter<bool>("send_gga", true);
    gga_period_s_ = declare_parameter<double>("gga_period_s", 10.0);
    reconnect_min_s_ = declare_parameter<double>("reconnect_backoff_s_min", 1.0);
    reconnect_max_s_ = declare_parameter<double>("reconnect_backoff_s_max", 30.0);
    // Stall watchdog: force a reconnect if no RTCM frame arrives for this
    // long while the socket is still "open" (half-open / idle VRS). 0 disables.
    rtcm_timeout_s_ = declare_parameter<double>("rtcm_timeout_s", 5.0);
    // Bound each TCP connect attempt so a dead network fails fast (0 = OS default).
    connect_timeout_s_ = declare_parameter<double>("connect_timeout_s", 5.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss_link");

    if (host_.empty() || mountpoint_.empty()) {
      RCLCPP_ERROR(get_logger(),
        "ntrip_client: 'host' and 'mountpoint' parameters are required.");
    }

    rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>("rtcm/out", rclcpp::QoS(50));
    if (send_gga_) {
      gga_sub_ = create_subscription<nmea_msgs::msg::Sentence>(
        "nmea_sentence", rclcpp::QoS(10),
        [this](const nmea_msgs::msg::Sentence::SharedPtr msg) {
          // Only stash GGA sentences (covers $GPGGA, $GNGGA, etc.).
          if (msg->sentence.size() >= 6 && msg->sentence.compare(3, 3, "GGA") == 0) {
            std::lock_guard<std::mutex> lock(gga_mutex_);
            last_gga_ = msg->sentence;
          }
        });
    }

    running_ = true;
    io_thread_ = std::thread([this] {run();});
  }

  ~NtripClientNode() override
  {
    running_ = false;
    const int fd = sock_fd_.exchange(-1);
    if (fd >= 0) {
      ::shutdown(fd, SHUT_RDWR);
    }
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  static int64_t steady_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_frame(const std::vector<uint8_t> & frame)
  {
    rtcm_msgs::msg::Message msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.message.assign(frame.begin(), frame.end());
    rtcm_pub_->publish(msg);
    last_frame_ns_.store(steady_ns(), std::memory_order_relaxed);
  }

  void interruptible_sleep(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
    while (running_ && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  std::string build_request(const std::string & version) const
  {
    std::string auth;
    if (!username_.empty() || !password_.empty()) {
      auth = "Authorization: Basic " + base64_encode(username_ + ":" + password_) + "\r\n";
    }
    std::ostringstream req;
    if (version == "1") {
      req << "GET /" << mountpoint_ << " HTTP/1.0\r\n"
          << "User-Agent: NTRIP " << user_agent_ << "\r\n"
          << "Accept: */*\r\n"
          << auth
          << "\r\n";
    } else {
      req << "GET /" << mountpoint_ << " HTTP/1.1\r\n"
          << "Host: " << host_ << ":" << port_ << "\r\n"
          << "Ntrip-Version: Ntrip/2.0\r\n"
          << "User-Agent: NTRIP " << user_agent_ << "\r\n"
          << auth
          << "Connection: close\r\n"
          << "\r\n";
    }
    return req.str();
  }

  /// Read response headers (everything up to and including "\r\n\r\n").
  /// Returns the header block on success; empty string on failure or if
  /// `running_` clears.
  std::string read_headers(int fd, std::vector<uint8_t> & leftover)
  {
    std::string header;
    header.reserve(1024);
    uint8_t buf[512];
    while (running_) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n <= 0) {
        if (n < 0 && errno == EINTR) {continue;}
        return {};
      }
      header.append(reinterpret_cast<char *>(buf), static_cast<std::size_t>(n));
      const auto pos = header.find("\r\n\r\n");
      if (pos != std::string::npos) {
        const std::size_t header_end = pos + 4;
        leftover.assign(header.begin() + header_end, header.end());
        header.resize(header_end);
        return header;
      }
      if (header.size() > 8192) {return {};}  // runaway headers
    }
    return {};
  }

  /// Classify NTRIP response. Returns "v1_ok", "v2_ok", "sourcetable",
  /// "unauthorized" or "error".
  static std::string classify_response(const std::string & header, bool & chunked)
  {
    chunked = false;
    if (header.empty()) {return "error";}
    const std::string lower = [&] {
        std::string s = header;
        std::transform(s.begin(), s.end(), s.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c));});
        return s;
      } ();
    if (lower.find("transfer-encoding: chunked") != std::string::npos) {
      chunked = true;
    }
    if (header.rfind("ICY 200 OK", 0) == 0) {return "v1_ok";}
    if (header.rfind("HTTP/1.1 200", 0) == 0 || header.rfind("HTTP/1.0 200", 0) == 0) {
      if (lower.find("content-type: gnss/data") != std::string::npos ||
        lower.find("ntrip-version") != std::string::npos)
      {
        return "v2_ok";
      }
      // Some casters return 200 with a sourcetable instead of failing.
      if (lower.find("content-type: gnss/sourcetable") != std::string::npos) {
        return "sourcetable";
      }
      return "v2_ok";
    }
    if (header.rfind("SOURCETABLE 200 OK", 0) == 0) {return "sourcetable";}
    if (header.find(" 401 ") != std::string::npos) {return "unauthorized";}
    return "error";
  }

  /// Read at most `len` bytes into `out`. Returns true on success.
  bool recv_exact(int fd, uint8_t * out, std::size_t len)
  {
    std::size_t got = 0;
    while (got < len && running_) {
      const ssize_t n = ::recv(fd, out + got, len - got, 0);
      if (n > 0) {
        got += static_cast<std::size_t>(n);
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        return false;
      }
    }
    return got == len;
  }

  /// Read a chunked-encoding chunk size line (hex), terminated by CRLF.
  /// Returns size in bytes, or SIZE_MAX on error.
  std::size_t read_chunk_size(int fd)
  {
    std::string line;
    while (running_ && line.size() < 32) {
      uint8_t c;
      if (!recv_exact(fd, &c, 1)) {return SIZE_MAX;}
      if (c == '\n' && !line.empty() && line.back() == '\r') {
        line.pop_back();
        break;
      }
      line.push_back(static_cast<char>(c));
    }
    if (line.empty()) {return SIZE_MAX;}
    try {
      return static_cast<std::size_t>(std::stoul(line, nullptr, 16));
    } catch (...) {
      return SIZE_MAX;
    }
  }

  /// Read body in chunked transfer-encoding, pushing bytes into framer.
  void read_chunked(int fd)
  {
    std::array<uint8_t, 4096> buf{};
    while (running_) {
      const std::size_t size = read_chunk_size(fd);
      if (size == SIZE_MAX) {return;}
      if (size == 0) {
        // trailer + final CRLF
        uint8_t crlf[2];
        recv_exact(fd, crlf, 2);
        return;
      }
      std::size_t left = size;
      while (left > 0 && running_) {
        const std::size_t want = std::min(left, buf.size());
        if (!recv_exact(fd, buf.data(), want)) {return;}
        framer_.push(buf.data(), want);
        left -= want;
      }
      uint8_t crlf[2];
      if (!recv_exact(fd, crlf, 2)) {return;}
    }
  }

  /// Read body as a raw byte stream until EOF.
  void read_raw(int fd, const std::vector<uint8_t> & leftover)
  {
    if (!leftover.empty()) {
      framer_.push(leftover.data(), leftover.size());
    }
    uint8_t buf[4096];
    while (running_) {
      const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
      if (n > 0) {
        framer_.push(buf, static_cast<std::size_t>(n));
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        return;
      }
    }
  }

  /// Attempt one full connect/handshake/stream session at the given
  /// version. Returns true if any RTCM was streamed (so the outer loop
  /// can reset backoff).
  bool one_session(const std::string & version)
  {
    std::string err;
    RCLCPP_INFO(get_logger(), "NTRIP v%s connecting to %s:%d/%s",
      version.c_str(), host_.c_str(), port_, mountpoint_.c_str());
    const int fd = tcp_connect(host_, port_, &err, connect_timeout_s_);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "Connect failed: %s", err.c_str());
      return false;
    }
    sock_fd_ = fd;

    const std::string req = build_request(version);
    if (!write_all(fd, req.data(), req.size())) {
      RCLCPP_WARN(get_logger(), "Failed to send NTRIP request.");
      close_socket();
      return false;
    }

    std::vector<uint8_t> leftover;
    const std::string header = read_headers(fd, leftover);
    bool chunked = false;
    const std::string kind = classify_response(header, chunked);
    if (kind != "v1_ok" && kind != "v2_ok") {
      RCLCPP_WARN(get_logger(), "NTRIP handshake rejected (%s).", kind.c_str());
      close_socket();
      return false;
    }
    RCLCPP_INFO(get_logger(), "NTRIP stream open (%s, chunked=%s).",
      kind.c_str(), chunked ? "yes" : "no");

    framer_.reset();
    // Send the first GGA immediately by backdating the timer. VRS / nearest-
    // base casters (e.g. PointOne "AUTO") only start streaming RTCM once they
    // have the client position, and some drop a connection that stays idle.
    // Waiting a full gga_period_s before the first upload would also let the
    // stall watchdog (rtcm_timeout_s) reconnect before any corrections arrive,
    // producing an endless connect->stall->reconnect loop with no RTCM.
    auto last_gga_send = std::chrono::steady_clock::now() -
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(gga_period_s_));
    bool streamed_any = false;
    const std::size_t before = framer_.frames_emitted();
    // Arm the stall watchdog: treat the stream as live as of now.
    last_frame_ns_.store(steady_ns(), std::memory_order_relaxed);

    // Spawn a tiny helper thread for the body read so we can interleave
    // GGA uploads on the main session thread.
    std::atomic<bool> body_done{false};
    std::thread reader([&] {
        if (chunked) {
          read_chunked(fd);
        } else {
          read_raw(fd, leftover);
        }
        body_done = true;
      });

    while (running_ && !body_done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // Stall watchdog: if no RTCM has arrived for rtcm_timeout_s_, shut the
      // socket so the reader's blocking recv() returns, ending this session so
      // the outer loop reconnects (guards against a half-open / idle stream).
      if (rtcm_timeout_s_ > 0.0) {
        const double idle_s =
          (steady_ns() - last_frame_ns_.load(std::memory_order_relaxed)) / 1e9;
        if (idle_s >= rtcm_timeout_s_) {
          RCLCPP_WARN(get_logger(),
            "No RTCM for %.1fs; forcing reconnect (stalled stream).", idle_s);
          const int fd_now = sock_fd_.load();
          if (fd_now >= 0) {::shutdown(fd_now, SHUT_RDWR);}
          break;
        }
      }
      if (send_gga_ && gga_period_s_ > 0.0) {
        const auto now_t = std::chrono::steady_clock::now();
        const double elapsed =
          std::chrono::duration<double>(now_t - last_gga_send).count();
        if (elapsed >= gga_period_s_) {
          std::string g;
          {
            std::lock_guard<std::mutex> lock(gga_mutex_);
            g = last_gga_;
          }
          if (!g.empty()) {
            if (g.back() != '\n') {g += "\r\n";}
            if (!write_all(fd, g.data(), g.size())) {
              RCLCPP_DEBUG(get_logger(), "GGA upload failed; will reconnect.");
            }
            last_gga_send = now_t;
          }
        }
      }
    }

    if (reader.joinable()) {reader.join();}
    streamed_any = framer_.frames_emitted() > before;
    close_socket();
    return streamed_any;
  }

  void close_socket()
  {
    const int fd = sock_fd_.exchange(-1);
    if (fd >= 0) {
      ::close(fd);
    }
  }

  void run()
  {
    double backoff = reconnect_min_s_;
    while (running_) {
      bool ok = false;
      if (ntrip_version_ == "1") {
        ok = one_session("1");
      } else if (ntrip_version_ == "2") {
        ok = one_session("2");
      } else {
        ok = one_session("2");
        if (!ok && running_) {
          RCLCPP_INFO(get_logger(), "Falling back to NTRIP v1.");
          ok = one_session("1");
        }
      }
      if (ok) {
        backoff = reconnect_min_s_;
      }
      if (!running_) {break;}
      RCLCPP_INFO(get_logger(), "Reconnecting in %.1fs", backoff);
      interruptible_sleep(backoff);
      backoff = std::min(backoff * 2.0, reconnect_max_s_);
    }
  }

  // Parameters
  std::string host_;
  int port_{2101};
  std::string mountpoint_;
  std::string username_;
  std::string password_;
  std::string ntrip_version_;
  std::string user_agent_;
  std::string frame_id_;
  bool send_gga_{true};
  double gga_period_s_{10.0};
  double reconnect_min_s_{1.0};
  double reconnect_max_s_{30.0};
  double rtcm_timeout_s_{5.0};
  double connect_timeout_s_{5.0};

  // Runtime
  std::mutex gga_mutex_;
  std::string last_gga_;
  RtcmFramer framer_;
  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr gga_sub_;
  std::atomic<bool> running_{false};
  std::atomic<int> sock_fd_{-1};
  std::atomic<int64_t> last_frame_ns_{0};
  std::thread io_thread_;
};

}  // namespace ntrip_client

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ntrip_client::NtripClientNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
