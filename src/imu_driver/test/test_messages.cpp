// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "imu_driver/messages.hpp"

using imu_driver::BitStatus;
using imu_driver::build_command;
using imu_driver::CalibHrData;
using imu_driver::Command;
using imu_driver::DevInfo;
using imu_driver::euler_from_quaternion_kernel;
using imu_driver::kCalibHrPayloadLen;
using imu_driver::parse_bit;
using imu_driver::parse_calib_hr;
using imu_driver::parse_dev_info;
using imu_driver::Quaternion;
using imu_driver::quaternion_from_euler_kernel;

namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

void put_i32(std::vector<uint8_t> & b, size_t off, int32_t v)
{
  auto u = static_cast<uint32_t>(v);
  b[off + 0] = static_cast<uint8_t>(u & 0xFF);
  b[off + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  b[off + 2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  b[off + 3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

void put_i16(std::vector<uint8_t> & b, size_t off, int16_t v)
{
  auto u = static_cast<uint16_t>(v);
  b[off + 0] = static_cast<uint8_t>(u & 0xFF);
  b[off + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}
}  // namespace

// --- Command framing (KERNEL ICD Appendix B exact bytes) --------------------

TEST(Messages, BuildCommandMatchesSpecBytes)
{
  EXPECT_EQ(
    build_command(Command::kCalibHR),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x81, 0x88, 0x00}));
  EXPECT_EQ(
    build_command(Command::kOrientation),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x33, 0x3A, 0x00}));
  EXPECT_EQ(
    build_command(Command::kStop),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xFE, 0x05, 0x01}));
  EXPECT_EQ(
    build_command(Command::kGetDevInfo),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x12, 0x19, 0x00}));
  EXPECT_EQ(
    build_command(Command::kGetBIT),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x1A, 0x21, 0x00}));
  EXPECT_EQ(
    build_command(Command::kQuatData),
    (std::vector<uint8_t>{0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x82, 0x89, 0x00}));
}

// --- CalibHR payload parsing ------------------------------------------------

TEST(Messages, ParseCalibHrScalesToSi)
{
  std::vector<uint8_t> p(kCalibHrPayloadLen, 0);
  put_i32(p, 0, 45000);       // heading 45.000 deg
  put_i32(p, 4, -10000);      // pitch -10.000 deg
  put_i32(p, 8, 20000);       // roll 20.000 deg
  put_i32(p, 12, 150000);     // gyroX 1.5 deg/s
  put_i32(p, 16, -200000);    // gyroY -2.0 deg/s
  put_i32(p, 20, 50000);      // gyroZ 0.5 deg/s
  put_i32(p, 24, 100000);     // accX 0.1 g
  put_i32(p, 28, -250000);    // accY -0.25 g
  put_i32(p, 32, 1000000);    // accZ 1.0 g
  // 36..41 mag = 0
  p[42] = 0xD2;
  p[43] = 0x04;               // counter = 1234
  put_i16(p, 46, 0);          // USW = 0
  put_i16(p, 50, 250);        // temper 25.0 C

  CalibHrData d;
  ASSERT_TRUE(parse_calib_hr(p.data(), p.size(), d));

  EXPECT_NEAR(d.heading_rad, 45.0 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.pitch_rad, -10.0 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.roll_rad, 20.0 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.angular_velocity[0], 1.5 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.angular_velocity[1], -2.0 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.angular_velocity[2], 0.5 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(d.linear_acceleration[0], 0.1 * 9.8106, 1e-6);
  EXPECT_NEAR(d.linear_acceleration[1], -0.25 * 9.8106, 1e-6);
  EXPECT_NEAR(d.linear_acceleration[2], 1.0 * 9.8106, 1e-6);
  EXPECT_EQ(d.counter, 1234u);
  EXPECT_NEAR(d.temperature_c, 25.0, 1e-9);
  EXPECT_FALSE(d.usw.sensors_comm_failure);
}

TEST(Messages, ParseCalibHrRejectsShortPayload)
{
  std::vector<uint8_t> p(kCalibHrPayloadLen - 1, 0);
  CalibHrData d;
  EXPECT_FALSE(parse_calib_hr(p.data(), p.size(), d));
}

// --- Quaternion <-> Euler (Appendix D) --------------------------------------

TEST(Messages, EulerQuaternionRoundTrip)
{
  for (double h = -170; h <= 170; h += 40) {
    for (double pitch = -80; pitch <= 80; pitch += 40) {
      for (double r = -170; r <= 170; r += 40) {
        const double hr = h * kDeg2Rad, pr = pitch * kDeg2Rad, rr = r * kDeg2Rad;
        Quaternion q = quaternion_from_euler_kernel(hr, pr, rr);
        double ho, po, ro;
        euler_from_quaternion_kernel(q, ho, po, ro);
        EXPECT_NEAR(ho, hr, 1e-6) << "h=" << h << " p=" << pitch << " r=" << r;
        EXPECT_NEAR(po, pr, 1e-6) << "h=" << h << " p=" << pitch << " r=" << r;
        EXPECT_NEAR(ro, rr, 1e-6) << "h=" << h << " p=" << pitch << " r=" << r;
      }
    }
  }
}

TEST(Messages, QuaternionIdentityForZeroEuler)
{
  Quaternion q = quaternion_from_euler_kernel(0, 0, 0);
  EXPECT_NEAR(q.w, 1.0, 1e-9);
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.z, 0.0, 1e-9);
}

TEST(Messages, QuaternionPureHeadingIsNegativeZ)
{
  // Clockwise-positive heading maps to a negative rotation about +Z.
  const double k = 90.0 * kDeg2Rad;
  Quaternion q = quaternion_from_euler_kernel(k, 0, 0);
  EXPECT_NEAR(q.w, std::cos(k / 2), 1e-9);
  EXPECT_NEAR(q.x, 0.0, 1e-9);
  EXPECT_NEAR(q.y, 0.0, 1e-9);
  EXPECT_NEAR(q.z, -std::sin(k / 2), 1e-9);
}

// --- Unit Status Word -------------------------------------------------------

TEST(Messages, DecodeUswFlags)
{
  using imu_driver::decode_usw;
  auto u = decode_usw((1U << 1) | (1U << 12) | (1U << 14));
  EXPECT_TRUE(u.sensors_comm_failure);
  EXPECT_FALSE(u.sensors_config_failure);
  EXPECT_TRUE(u.ang_rate_z_out_of_range);
  EXPECT_FALSE(u.ang_rate_x_out_of_range);
  EXPECT_TRUE(u.temperature_out_of_range);
  EXPECT_EQ(u.raw, (1U << 1) | (1U << 12) | (1U << 14));
}

// --- GetDevInfo / GetBIT ----------------------------------------------------

TEST(Messages, ParseDevInfo)
{
  std::vector<uint8_t> p(166, 0);
  const char * sn = "SN123456";
  for (int i = 0; i < 8; ++i) {
    p[i] = static_cast<uint8_t>(sn[i]);
                                                                  }
  const char * fw = "1.39";
  for (int i = 0; i < 4; ++i) {
    p[8 + i] = static_cast<uint8_t>(fw[i]);
                                                                      }
  p[49] = 110;  // IMU type

  DevInfo info;
  ASSERT_TRUE(parse_dev_info(p.data(), p.size(), info));
  EXPECT_EQ(info.serial, "SN123456");
  EXPECT_EQ(info.firmware, "1.39");
  EXPECT_EQ(info.imu_type, 110u);
}

TEST(Messages, ParseBitHealthy)
{
  std::vector<uint8_t> p = {0x00, 0x00, 0x00, 0x00};
  BitStatus s;
  ASSERT_TRUE(parse_bit(p.data(), p.size(), s));
  EXPECT_TRUE(s.accel_ok);
  EXPECT_TRUE(s.gyro_ok);
  EXPECT_TRUE(s.flash_ok);
}

TEST(Messages, ParseBitFlagsFlashFault)
{
  // bit 12 set -> flash check fault.
  std::vector<uint8_t> p = {0x00, 0x10, 0x00, 0x00};
  BitStatus s;
  ASSERT_TRUE(parse_bit(p.data(), p.size(), s));
  EXPECT_FALSE(s.flash_ok);
  EXPECT_TRUE(s.accel_ok);
  EXPECT_TRUE(s.gyro_ok);
}
