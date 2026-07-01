// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef NTRIP_CLIENT__POSIX_IO_HPP_
#define NTRIP_CLIENT__POSIX_IO_HPP_

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

namespace ntrip_client
{

/// RAII wrapper for a POSIX file descriptor.
class ScopedFd
{
public:
  ScopedFd() = default;
  explicit ScopedFd(int fd)
  : fd_(fd) {}
  ~ScopedFd() {close();}

  ScopedFd(const ScopedFd &) = delete;
  ScopedFd & operator=(const ScopedFd &) = delete;

  ScopedFd(ScopedFd && other) noexcept
  : fd_(other.fd_) {other.fd_ = -1;}
  ScopedFd & operator=(ScopedFd && other) noexcept
  {
    if (this != &other) {
      close();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  int get() const noexcept {return fd_;}
  bool valid() const noexcept {return fd_ >= 0;}
  int release() noexcept
  {
    const int f = fd_;
    fd_ = -1;
    return f;
  }
  void reset(int fd = -1) noexcept
  {
    close();
    fd_ = fd;
  }
  void close() noexcept
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_{-1};
};

/// Connect to host:port over TCP. Returns a connected fd or -1 on failure
/// (errno set). `err_out` receives a human-readable error string when
/// non-null. `connect_timeout_s` bounds the connect via a non-blocking
/// connect + poll() so a dead network fails fast instead of blocking for the
/// full OS SYN timeout (<= 0 keeps the OS default blocking behaviour).
inline int tcp_connect(
  const std::string & host, int port, std::string * err_out = nullptr,
  double connect_timeout_s = 5.0)
{
  struct addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * res = nullptr;
  const std::string port_str = std::to_string(port);
  const int gai_rc = ::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res);
  if (gai_rc != 0) {
    if (err_out) {*err_out = std::string("getaddrinfo: ") + ::gai_strerror(gai_rc);}
    return -1;
  }

  int fd = -1;
  for (auto * ai = res; ai != nullptr; ai = ai->ai_next) {
    fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0) {continue;}

    if (connect_timeout_s <= 0.0) {
      // Blocking connect (OS default timeout).
      if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {break;}
      ::close(fd);
      fd = -1;
      continue;
    }

    // Non-blocking connect bounded by poll().
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(fd, ai->ai_addr, ai->ai_addrlen);
    if (rc < 0 && errno == EINPROGRESS) {
      struct pollfd pfd{fd, POLLOUT, 0};
      const int pr = ::poll(&pfd, 1, static_cast<int>(connect_timeout_s * 1000.0));
      if (pr > 0 && (pfd.revents & POLLOUT)) {
        int soerr = 0;
        socklen_t len = sizeof(soerr);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0) {
          rc = 0;
        } else {
          errno = soerr != 0 ? soerr : ETIMEDOUT;
          rc = -1;
        }
      } else {
        if (pr == 0) {errno = ETIMEDOUT;}
        rc = -1;
      }
    }
    if (rc == 0) {
      ::fcntl(fd, F_SETFL, flags);  // restore blocking mode
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(res);
  if (fd < 0 && err_out) {
    *err_out = std::string("connect: ") + std::strerror(errno);
  }
  return fd;
}

/// Listen on bind_address:port for IPv4. Returns the listen fd or -1 (errno set).
inline int tcp_listen(const std::string & bind_address, int port, std::string * err_out = nullptr)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    if (err_out) {*err_out = std::string("socket: ") + std::strerror(errno);}
    return -1;
  }
  const int yes = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (bind_address.empty() || bind_address == "0.0.0.0") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
    if (err_out) {*err_out = "invalid bind_address (IPv4 only)";}
    ::close(fd);
    return -1;
  }

  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    if (err_out) {*err_out = std::string("bind: ") + std::strerror(errno);}
    ::close(fd);
    return -1;
  }
  if (::listen(fd, 1) < 0) {
    if (err_out) {*err_out = std::string("listen: ") + std::strerror(errno);}
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace ntrip_client

#endif  // NTRIP_CLIENT__POSIX_IO_HPP_
