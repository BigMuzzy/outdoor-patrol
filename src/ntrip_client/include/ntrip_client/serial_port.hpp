// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef NTRIP_CLIENT__SERIAL_PORT_HPP_
#define NTRIP_CLIENT__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ntrip_client
{

/// Map an integer baud rate to its `termios` constant.
inline speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return B0;
  }
}

/// Open a serial port at the given baud rate (8N1, raw, no flow control).
/// Returns the file descriptor or -1 on failure. `err_out` (optional) gets
/// a human-readable error.
inline int open_serial(
  const std::string & port, int baud, std::string * err_out = nullptr)
{
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0) {
    if (err_out) {*err_out = std::string("open: ") + std::strerror(errno);}
    return -1;
  }
  struct termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    if (err_out) {*err_out = std::string("tcgetattr: ") + std::strerror(errno);}
    ::close(fd);
    return -1;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  // Blocking read of >=1 byte, no inter-byte timeout.
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;

  const speed_t spd = baud_to_speed(baud);
  if (spd == B0) {
    if (err_out) {*err_out = "unsupported baud rate";}
    ::close(fd);
    return -1;
  }
  ::cfsetispeed(&tio, spd);
  ::cfsetospeed(&tio, spd);
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    if (err_out) {*err_out = std::string("tcsetattr: ") + std::strerror(errno);}
    ::close(fd);
    return -1;
  }
  ::tcflush(fd, TCIOFLUSH);
  return fd;
}

}  // namespace ntrip_client

#endif  // NTRIP_CLIENT__SERIAL_PORT_HPP_
