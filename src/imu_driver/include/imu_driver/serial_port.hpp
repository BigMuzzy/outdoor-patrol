// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef IMU_DRIVER__SERIAL_PORT_HPP_
#define IMU_DRIVER__SERIAL_PORT_HPP_

#include <termios.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace imu_driver
{

/// Convert an integer baudrate to a termios speed_t. Returns B0 if the
/// rate is not supported by termios on this platform.
speed_t baud_to_speed(int baud);

/// Open a serial port in raw 8N1 mode, no flow control.
/// Returns a non-negative fd, or -1 with the system error placed in
/// `*err_out`. The fd is opened with `O_NOCTTY` and is blocking
/// (VMIN=1, VTIME=0) by default.
int open_serial(const std::string & port, int baud, std::string * err_out);

}  // namespace imu_driver

#endif  // IMU_DRIVER__SERIAL_PORT_HPP_
