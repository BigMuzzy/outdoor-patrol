// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef IMU_DRIVER__MESSAGES_HPP_
#define IMU_DRIVER__MESSAGES_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace imu_driver
{

/// Inertial Labs control command codes (KERNEL ICD Table 6.15 / Appendix B).
enum class Command : uint8_t
{
  kOrientation = 0x33,
  kGAData = 0x8F,
  kGAmData = 0x9B,
  kGAAData = 0xA5,
  kGAAmData = 0xA6,
  kQuatData = 0x82,
  kCalibHR = 0x81,
  kUserDefData = 0x95,
  kUserDefDataConfig = 0x96,
  kGetUserDefDataStruct = 0x97,
  kGetDevInfo = 0x12,
  kGetBIT = 0x1A,
  kStop = 0xFE,
};

constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;

/// g as defined by the KERNEL ICD (Section 6.2, Note 1), in m/s^2.
constexpr double kGravity = 9.8106;

/// Payload length of the "Calibrated HR Data" format (Table 6.9), in bytes.
constexpr size_t kCalibHrPayloadLen = 52;

/// A unit quaternion. `w` is the real part; (x, y, z) the vector part.
struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Decoded Unit Status Word (KERNEL ICD Table 6.25). `raw` keeps the original
/// 16-bit value; the booleans flag fault conditions (true == fault / warning).
struct Usw
{
  uint16_t raw{0};
  bool sensors_comm_failure{false};      ///< bit 1
  bool sensors_config_failure{false};    ///< bit 2
  bool ang_rate_x_out_of_range{false};   ///< bit 10
  bool ang_rate_y_out_of_range{false};   ///< bit 11
  bool ang_rate_z_out_of_range{false};   ///< bit 12
  bool temperature_out_of_range{false};  ///< bit 14
};

/// Fully decoded "Calibrated HR Data" sample in SI units, in the IMU's own
/// sensor axes (X = lateral/right, Y = longitudinal/forward, Z = normal/up).
struct CalibHrData
{
  double heading_rad{0.0};   ///< Relative heading (no magnetometer).
  double pitch_rad{0.0};
  double roll_rad{0.0};
  Quaternion orientation;    ///< Body->world, derived from the Euler angles.
  double angular_velocity[3]{0.0, 0.0, 0.0};     ///< rad/s, sensor axes.
  double linear_acceleration[3]{0.0, 0.0, 0.0};  ///< m/s^2, sensor axes.
  uint16_t counter{0};       ///< 2 kHz sample counter.
  Usw usw;
  double temperature_c{0.0};
};

/// Decoded GetDevInfo response (KERNEL ICD Table 6.16).
struct DevInfo
{
  std::string serial;
  std::string firmware;
  uint8_t imu_type{0};
};

/// Decoded GetBIT response (KERNEL ICD Table 6.17).
struct BitStatus
{
  uint32_t raw{0};
  bool accel_ok{true};
  bool gyro_ok{true};
  bool flash_ok{true};
};

/// 16-bit arithmetic checksum (sum of bytes), as used by the framing.
uint16_t checksum(const uint8_t * data, size_t len);

/// Build a single-payload-byte command frame: AA 55 00 00 07 00 <code> ck ck.
std::vector<uint8_t> build_command(Command code);

/// Parse a "Calibrated HR Data" payload into SI units. Returns false if the
/// payload length is wrong. `gravity` converts g -> m/s^2.
bool parse_calib_hr(
  const uint8_t * payload, size_t len, CalibHrData & out, double gravity = kGravity);

/// Parse a GetDevInfo payload. Returns false if too short.
bool parse_dev_info(const uint8_t * payload, size_t len, DevInfo & out);

/// Parse a GetBIT payload. Returns false if too short.
bool parse_bit(const uint8_t * payload, size_t len, BitStatus & out);

/// Decode a raw Unit Status Word into named fault flags.
Usw decode_usw(uint16_t raw);

/// Convert KERNEL Euler angles (heading/pitch/roll, radians) into the
/// body->world quaternion the device itself reports (KERNEL ICD Appendix D):
/// q = qz(-heading) * qx(pitch) * qy(roll).
Quaternion quaternion_from_euler_kernel(double heading_rad, double pitch_rad, double roll_rad);

/// Inverse of quaternion_from_euler_kernel(), implementing KERNEL ICD
/// equation (D.7). Provided for verification and downstream use.
void euler_from_quaternion_kernel(
  const Quaternion & q, double & heading_rad, double & pitch_rad, double & roll_rad);

}  // namespace imu_driver

#endif  // IMU_DRIVER__MESSAGES_HPP_
