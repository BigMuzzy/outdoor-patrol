// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace um982_driver
{

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return B0;
  }
}

int open_serial(const std::string & port, int baud, std::string * err_out)
{
  auto set_err = [&](const std::string & m) {
      if (err_out) {*err_out = m;}
    };
  speed_t speed = baud_to_speed(baud);
  if (speed == B0) {
    set_err("unsupported baudrate: " + std::to_string(baud));
    return -1;
  }
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd < 0) {
    set_err(std::string("open ") + port + ": " + std::strerror(errno));
    return -1;
  }
  struct termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    set_err(std::string("tcgetattr: ") + std::strerror(errno));
    ::close(fd);
    return -1;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~PARENB;     // no parity
  tio.c_cflag &= ~CSTOPB;     // 1 stop bit
  tio.c_cflag &= ~CRTSCTS;    // no RTS/CTS
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);  // no soft flow control
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
    set_err(std::string("cfsetspeed: ") + std::strerror(errno));
    ::close(fd);
    return -1;
  }
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    set_err(std::string("tcsetattr: ") + std::strerror(errno));
    ::close(fd);
    return -1;
  }
  ::tcflush(fd, TCIOFLUSH);
  return fd;
}

}  // namespace um982_driver
