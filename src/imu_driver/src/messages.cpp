// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "imu_driver/messages.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace imu_driver
{

namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

int16_t read_i16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

uint16_t read_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

int32_t read_i32(const uint8_t * p)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24));
}

// Hamilton product a * b for body->world composition.
Quaternion qmul(const Quaternion & a, const Quaternion & b)
{
  Quaternion r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

// Extract a fixed-width C string field; stops at the first NUL and trims
// trailing whitespace.
std::string read_cstr(const uint8_t * p, size_t n)
{
  size_t real = 0;
  while (real < n && p[real] != 0) {++real;}
  std::string s(reinterpret_cast<const char *>(p), real);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' ||
    s.back() == '\r' || s.back() == '\n'))
  {
    s.pop_back();
  }
  return s;
}
}  // namespace

uint16_t checksum(const uint8_t * data, size_t len)
{
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum = static_cast<uint16_t>(sum + data[i]);
  }
  return sum;
}

std::vector<uint8_t> build_command(Command code)
{
  // AA 55 | msg_type=0 | data_id=0 | length=0x0007 | payload=code | checksum
  std::vector<uint8_t> f = {
    kHeader0, kHeader1,
    0x00,                 // msg_type = command
    0x00,                 // data_id
    0x07, 0x00,           // length = payload(1) + 6
    static_cast<uint8_t>(code),
  };
  const uint16_t ck = checksum(f.data() + 2, f.size() - 2);
  f.push_back(static_cast<uint8_t>(ck & 0xFF));
  f.push_back(static_cast<uint8_t>((ck >> 8) & 0xFF));
  return f;
}

Usw decode_usw(uint16_t raw)
{
  Usw u;
  u.raw = raw;
  u.sensors_comm_failure = (raw & (1U << 1)) != 0;
  u.sensors_config_failure = (raw & (1U << 2)) != 0;
  u.ang_rate_x_out_of_range = (raw & (1U << 10)) != 0;
  u.ang_rate_y_out_of_range = (raw & (1U << 11)) != 0;
  u.ang_rate_z_out_of_range = (raw & (1U << 12)) != 0;
  u.temperature_out_of_range = (raw & (1U << 14)) != 0;
  return u;
}

bool parse_calib_hr(const uint8_t * payload, size_t len, CalibHrData & out, double gravity)
{
  if (len < kCalibHrPayloadLen) {
    return false;
  }
  // Table 6.9 layout (little-endian):
  //  0..3 Heading deg*1000, 4..7 Pitch deg*1000, 8..11 Roll deg*1000,
  //  12..23 Gyro XYZ deg/s*1e5, 24..35 Acc XYZ g*1e6, 36..41 Mag (=0),
  //  42..43 Counter, 44..45 Reserved, 46..47 USW, 48..49 Reserved,
  //  50..51 Temper degC*10.
  out.heading_rad = (read_i32(payload + 0) / 1000.0) * kDeg2Rad;
  out.pitch_rad = (read_i32(payload + 4) / 1000.0) * kDeg2Rad;
  out.roll_rad = (read_i32(payload + 8) / 1000.0) * kDeg2Rad;
  out.orientation = quaternion_from_euler_kernel(out.heading_rad, out.pitch_rad, out.roll_rad);

  for (int i = 0; i < 3; ++i) {
    out.angular_velocity[i] = (read_i32(payload + 12 + i * 4) / 1.0e5) * kDeg2Rad;
    out.linear_acceleration[i] = (read_i32(payload + 24 + i * 4) / 1.0e6) * gravity;
  }

  out.counter = read_u16(payload + 42);
  out.usw = decode_usw(read_u16(payload + 46));
  out.temperature_c = read_i16(payload + 50) / 10.0;
  return true;
}

bool parse_dev_info(const uint8_t * payload, size_t len, DevInfo & out)
{
  // Table 6.16: 0..7 serial, 8..47 firmware, 49 IMU type.
  if (len < 50) {
    return false;
  }
  out.serial = read_cstr(payload + 0, 8);
  out.firmware = read_cstr(payload + 8, 40);
  out.imu_type = payload[49];
  return true;
}

bool parse_bit(const uint8_t * payload, size_t len, BitStatus & out)
{
  // Table 6.17: low byte holds accel (bits 0-1), gyro (bits 2-7),
  // flash check (bit 12). 0 == OK (accel also accepts 2 as OK).
  if (len < 4) {
    return false;
  }
  out.raw =
    static_cast<uint32_t>(payload[0]) |
    (static_cast<uint32_t>(payload[1]) << 8) |
    (static_cast<uint32_t>(payload[2]) << 16) |
    (static_cast<uint32_t>(payload[3]) << 24);
  const uint8_t accel_field = static_cast<uint8_t>(payload[0] & 0x03);
  out.accel_ok = (accel_field == 0 || accel_field == 2);
  out.gyro_ok = (payload[0] & 0xFC) == 0;
  out.flash_ok = (payload[1] & (1U << (12 - 8))) == 0;
  return true;
}

Quaternion quaternion_from_euler_kernel(double heading_rad, double pitch_rad, double roll_rad)
{
  // q = qz(-heading) * qx(pitch) * qy(roll), reproducing the device's own
  // Euler<->quaternion relationship (KERNEL ICD Appendix D, eq. D.7).
  const double hz = -heading_rad * 0.5;
  const double hx = pitch_rad * 0.5;
  const double hy = roll_rad * 0.5;
  const Quaternion qz{std::cos(hz), 0.0, 0.0, std::sin(hz)};
  const Quaternion qx{std::cos(hx), std::sin(hx), 0.0, 0.0};
  const Quaternion qy{std::cos(hy), 0.0, std::sin(hy), 0.0};
  Quaternion q = qmul(qmul(qz, qx), qy);

  // Normalise to guard against accumulated rounding.
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n > 0.0) {
    q.w /= n;
    q.x /= n;
    q.y /= n;
    q.z /= n;
  }
  return q;
}

void euler_from_quaternion_kernel(
  const Quaternion & q, double & heading_rad, double & pitch_rad, double & roll_rad)
{
  const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  heading_rad = std::atan2(
    2.0 * (q1 * q2 - q0 * q3),
    q0 * q0 + q2 * q2 - q1 * q1 - q3 * q3);
  double sp = 2.0 * (q2 * q3 + q0 * q1);
  sp = std::max(-1.0, std::min(1.0, sp));
  pitch_rad = std::asin(sp);
  roll_rad = std::atan2(
    -2.0 * (q1 * q3 - q0 * q2),
    q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2);
}

}  // namespace imu_driver
