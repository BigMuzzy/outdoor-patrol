// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/command_builder.hpp"

#include <array>
#include <cstdio>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>

namespace um982_driver
{

namespace
{

std::array<uint32_t, 256> make_crc32_table()
{
  std::array<uint32_t, 256> t{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) {
      c = (c & 1U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
    }
    t[i] = c;
  }
  return t;
}

const std::array<uint32_t, 256> & crc32_table()
{
  static const auto t = make_crc32_table();
  return t;
}

std::string fmt_double(double v, int precision)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, v);
  return std::string(buf);
}

// Format an output rate as the Unicore manual documents it: bare integers or
// short decimals (1, 0.5, 0.2, 0.1, 0.05, 0.02 for 1..50 Hz), with no trailing
// zeros that the command parser would otherwise have to tolerate.
std::string fmt_rate(double seconds)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", seconds);
  return std::string(buf);
}

}  // namespace

uint32_t unicore_crc32(const uint8_t * data, size_t len)
{
  const auto & t = crc32_table();
  uint32_t c = 0xFFFFFFFFU;
  for (size_t i = 0; i < len; ++i) {
    c = t[(c ^ data[i]) & 0xFFU] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFU;
}

std::string format_unicore_command(std::string_view body)
{
  uint32_t crc = unicore_crc32(
    reinterpret_cast<const uint8_t *>(body.data()), body.size());
  char tail[16];
  std::snprintf(tail, sizeof(tail), "*%08x\r\n", crc);
  std::string out;
  out.reserve(body.size() + sizeof(tail));
  out.append(body.data(), body.size());
  out.append(tail);
  return out;
}

std::string build_mode_rover(std::string_view dynamics)
{
  std::string body = "MODE ROVER";
  if (!dynamics.empty()) {
    body.push_back(' ');
    body.append(dynamics.data(), dynamics.size());
  }
  return format_unicore_command(body);
}

std::string build_mode_base_fixed(double lat_deg, double lon_deg, double height_m)
{
  std::string body = "MODE BASE ";
  body += fmt_double(lat_deg, 9);
  body.push_back(' ');
  body += fmt_double(lon_deg, 9);
  body.push_back(' ');
  body += fmt_double(height_m, 3);
  return format_unicore_command(body);
}

std::string build_mode_base_survey(double averaging_s, double dist_m)
{
  std::string body = "MODE BASE TIME ";
  body += fmt_double(averaging_s, 1);
  if (dist_m > 0.0) {
    body.push_back(' ');
    body += fmt_double(dist_m, 2);
  }
  return format_unicore_command(body);
}

std::string build_mode_heading2(std::string_view mode)
{
  std::string body = "MODE HEADING2";
  if (!mode.empty()) {
    body.push_back(' ');
    body.append(mode.data(), mode.size());
  }
  return format_unicore_command(body);
}

std::string build_rtcm_output(int rtcm_id, std::string_view com, double period_s)
{
  char head[32];
  std::snprintf(head, sizeof(head), "rtcm%d ", rtcm_id);
  std::string body = head;
  body.append(com.data(), com.size());
  body.push_back(' ');
  body += fmt_double(period_s, 2);
  return format_unicore_command(body);
}

std::string build_log(std::string_view message, std::string_view com, double period_s)
{
  // Unicore data-output shorthand: "<MESSAGE> [PORT] <rate>". Omitting the
  // port targets the *current* port (the one the command arrived on), which is
  // what we want when driving the receiver over its USB/Type-C link — the
  // board's USB bridge is wired to a UART whose name we don't know a priori.
  // NMEA names must be GP-prefixed (GPGGA, GPRMC, ...); the receiver still
  // emits $GN... sentences. The earlier "log <com> <msg> ontime <p>" form sent
  // unprefixed names to a hardcoded COM and produced no output.
  std::string body;
  body.append(message.data(), message.size());
  if (!com.empty()) {
    body.push_back(' ');
    body.append(com.data(), com.size());
  }
  body.push_back(' ');
  body += fmt_rate(period_s);
  return format_unicore_command(body);
}

std::string build_unlogall()
{
  return format_unicore_command("unlogall");
}

std::string build_saveconfig()
{
  return format_unicore_command("saveconfig");
}

std::string build_antenna_delta_hen(double height, double east, double north)
{
  std::string body = "ANTENNADELTAHEN ";
  body += fmt_double(height, 4);
  body.push_back(' ');
  body += fmt_double(east, 4);
  body.push_back(' ');
  body += fmt_double(north, 4);
  return format_unicore_command(body);
}

}  // namespace um982_driver
