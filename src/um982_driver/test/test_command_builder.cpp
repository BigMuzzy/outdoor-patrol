// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "um982_driver/command_builder.hpp"

using um982_driver::build_log;
using um982_driver::build_mode_base_fixed;
using um982_driver::build_mode_heading2;
using um982_driver::build_mode_rover;
using um982_driver::build_rtcm_output;
using um982_driver::build_saveconfig;
using um982_driver::build_unlogall;
using um982_driver::format_unicore_command;
using um982_driver::unicore_crc32;

// Bit-loop reference CRC32 (zlib) for cross-check against the table-driven impl.
static uint32_t crc32_reference(const uint8_t * d, size_t n)
{
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= d[i];
    for (int k = 0; k < 8; ++k) {
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  return c ^ 0xFFFFFFFFu;
}

TEST(CommandBuilder, Crc32MatchesReferenceImpl)
{
  const std::string inputs[] = {
    "", "a", "MODE ROVER", "MODE BASE TIME 60.0 1.50", "saveconfig",
    "GPGGA com2 0.2",
  };
  for (const auto & s : inputs) {
    auto * p = reinterpret_cast<const uint8_t *>(s.data());
    EXPECT_EQ(unicore_crc32(p, s.size()), crc32_reference(p, s.size()))
      << "input=" << s;
  }
}

TEST(CommandBuilder, Crc32KnownValueForAscii123456789)
{
  const std::string s = "123456789";
  // zlib CRC32("123456789") == 0xcbf43926.
  EXPECT_EQ(
    unicore_crc32(reinterpret_cast<const uint8_t *>(s.data()), s.size()),
    0xcbf43926u);
}

TEST(CommandBuilder, FormatSendsBareCommandWithCrLf)
{
  // Unicore input commands carry no checksum; the receiver rejects a '*<crc>'
  // suffix. format_unicore_command() emits just "<body>\r\n".
  auto s = format_unicore_command("MODE ROVER");
  EXPECT_EQ(s, "MODE ROVER\r\n");
  EXPECT_EQ(s.find('*'), std::string::npos);
}

TEST(CommandBuilder, BuildersProduceExpectedPrefixes)
{
  EXPECT_EQ(build_mode_rover("UAV").substr(0, 14), "MODE ROVER UAV");
  EXPECT_EQ(build_mode_rover("").substr(0, 10), "MODE ROVER");
  EXPECT_EQ(build_mode_base_fixed(37.0, -121.0, 50.0).substr(0, 10), "MODE BASE ");
  EXPECT_EQ(build_mode_heading2("FIXLENGTH").substr(0, 23), "MODE HEADING2 FIXLENGTH");
  EXPECT_EQ(build_rtcm_output(1074, "com2", 1.0).substr(0, 15), "rtcm1074 com2 1");
  // Shorthand "<MESSAGE> [PORT] <rate>"; empty com -> current port.
  EXPECT_EQ(build_log("GPGGA", "", 0.2).substr(0, 9), "GPGGA 0.2");
  EXPECT_EQ(build_log("GPGGA", "com2", 1.0).substr(0, 12), "GPGGA com2 1");
  EXPECT_EQ(build_unlogall().substr(0, 8), "unlogall");
  EXPECT_EQ(build_saveconfig().substr(0, 10), "saveconfig");
}
