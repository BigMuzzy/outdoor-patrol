// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/nmea_parser.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace um982_driver
{

namespace
{

constexpr double kKnotsToMps = 0.5144444444444444;

std::string_view trim_eol(std::string_view s)
{
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
    s.remove_suffix(1);
  }
  return s;
}

std::optional<double> parse_double(std::string_view s)
{
  if (s.empty()) {
    return std::nullopt;
  }
  // std::from_chars for double is C++17 but not available in libstdc++
  // for floats until later. Fall back to strtod via a small buffer.
  char buf[64];
  if (s.size() >= sizeof(buf)) {
    return std::nullopt;
  }
  std::memcpy(buf, s.data(), s.size());
  buf[s.size()] = '\0';
  char * end = nullptr;
  double v = std::strtod(buf, &end);
  if (end == buf) {
    return std::nullopt;
  }
  return v;
}

std::optional<int64_t> parse_long(std::string_view s)
{
  if (s.empty()) {
    return std::nullopt;
  }
  int64_t v = 0;
  auto res = std::from_chars(s.data(), s.data() + s.size(), v);
  if (res.ec != std::errc()) {
    return std::nullopt;
  }
  return v;
}

/// Convert NMEA `DDMM.MMMM` / `DDDMM.MMMM` to signed decimal degrees,
/// applying the N/S/E/W hemisphere. Returns nullopt if input is empty.
std::optional<double> parse_latlon(std::string_view ddmm, std::string_view hemi)
{
  if (ddmm.empty() || hemi.empty()) {
    return std::nullopt;
  }
  auto raw = parse_double(ddmm);
  if (!raw) {
    return std::nullopt;
  }
  double whole_min = *raw;
  // DDMM.MMMM: degrees are everything left of the last two integer digits.
  double degrees = std::floor(whole_min / 100.0);
  double minutes = whole_min - degrees * 100.0;
  double decimal = degrees + minutes / 60.0;
  char h = static_cast<char>(std::toupper(static_cast<unsigned char>(hemi[0])));
  if (h == 'S' || h == 'W') {
    decimal = -decimal;
  }
  return decimal;
}

}  // namespace

bool verify_nmea_checksum(std::string_view sentence)
{
  sentence = trim_eol(sentence);
  if (sentence.size() < 4) {
    return false;
  }
  if (sentence.front() != '$' && sentence.front() != '!') {
    return false;
  }
  auto star = sentence.rfind('*');
  if (star == std::string_view::npos || star + 3 != sentence.size()) {
    return false;
  }
  uint8_t computed = 0;
  for (size_t i = 1; i < star; ++i) {
    computed ^= static_cast<uint8_t>(sentence[i]);
  }
  // Parse two hex digits.
  auto hex_val = [](char c) -> int {
      if (c >= '0' && c <= '9') {return c - '0';}
      if (c >= 'a' && c <= 'f') {return 10 + (c - 'a');}
      if (c >= 'A' && c <= 'F') {return 10 + (c - 'A');}
      return -1;
    };
  int hi = hex_val(sentence[star + 1]);
  int lo = hex_val(sentence[star + 2]);
  if (hi < 0 || lo < 0) {
    return false;
  }
  uint8_t expected = static_cast<uint8_t>((hi << 4) | lo);
  return computed == expected;
}

std::vector<std::string> split_nmea_fields(std::string_view sentence)
{
  sentence = trim_eol(sentence);
  if (sentence.size() < 4) {
    return {};
  }
  if (sentence.front() != '$' && sentence.front() != '!') {
    return {};
  }
  auto star = sentence.rfind('*');
  if (star == std::string_view::npos) {
    return {};
  }
  // Strip leading '$' and trailing '*HH'.
  auto body = sentence.substr(1, star - 1);
  std::vector<std::string> fields;
  size_t start = 0;
  for (size_t i = 0; i <= body.size(); ++i) {
    if (i == body.size() || body[i] == ',') {
      fields.emplace_back(body.substr(start, i - start));
      start = i + 1;
    }
  }
  return fields;
}

std::optional<NmeaGga> parse_gga(std::string_view sentence)
{
  auto fields = split_nmea_fields(sentence);
  if (fields.size() < 15) {
    return std::nullopt;
  }
  // Identifier is fields[0]. Last 3 chars must be "GGA".
  const std::string & id = fields[0];
  if (id.size() < 3 || id.compare(id.size() - 3, 3, "GGA") != 0) {
    return std::nullopt;
  }
  NmeaGga gga;
  gga.utc = fields[1];
  auto lat = parse_latlon(fields[2], fields[3]);
  auto lon = parse_latlon(fields[4], fields[5]);
  if (!lat || !lon) {
    // Missing position is allowed when quality==0; default to 0.
    gga.latitude_deg = 0.0;
    gga.longitude_deg = 0.0;
  } else {
    gga.latitude_deg = *lat;
    gga.longitude_deg = *lon;
  }
  if (auto q = parse_long(fields[6])) {
    if (*q >= 0 && *q <= 8) {
      gga.quality = static_cast<NmeaFixQuality>(*q);
    }
  }
  if (auto n = parse_long(fields[7])) {
    if (*n >= 0 && *n < 256) {
      gga.num_satellites = static_cast<uint16_t>(*n);
    }
  }
  if (auto h = parse_double(fields[8])) {
    gga.hdop = *h;
  }
  if (auto a = parse_double(fields[9])) {
    gga.altitude_m = *a;
  }
  if (auto g = parse_double(fields[11])) {
    gga.geoid_separation_m = *g;
  }
  if (auto age = parse_double(fields[13])) {
    gga.age_of_corrections_s = *age;
  }
  if (auto st = parse_long(fields[14])) {
    if (*st >= 0 && *st <= 4095) {
      gga.station_id = static_cast<uint16_t>(*st);
    }
  }
  return gga;
}

std::optional<NmeaRmc> parse_rmc(std::string_view sentence)
{
  auto fields = split_nmea_fields(sentence);
  if (fields.size() < 10) {
    return std::nullopt;
  }
  const std::string & id = fields[0];
  if (id.size() < 3 || id.compare(id.size() - 3, 3, "RMC") != 0) {
    return std::nullopt;
  }
  NmeaRmc rmc;
  rmc.utc = fields[1];
  rmc.valid = !fields[2].empty() && (fields[2][0] == 'A' || fields[2][0] == 'a');
  if (auto lat = parse_latlon(fields[3], fields[4])) {
    rmc.latitude_deg = *lat;
  }
  if (auto lon = parse_latlon(fields[5], fields[6])) {
    rmc.longitude_deg = *lon;
  }
  if (auto sog = parse_double(fields[7])) {
    rmc.speed_over_ground_mps = *sog * kKnotsToMps;
  }
  if (auto cog = parse_double(fields[8])) {
    rmc.course_over_ground_deg = *cog;
  }
  rmc.date_ddmmyy = fields[9];
  return rmc;
}

std::optional<NmeaVtg> parse_vtg(std::string_view sentence)
{
  auto fields = split_nmea_fields(sentence);
  // VTG has 9 or 10 fields including identifier.
  if (fields.size() < 9) {
    return std::nullopt;
  }
  const std::string & id = fields[0];
  if (id.size() < 3 || id.compare(id.size() - 3, 3, "VTG") != 0) {
    return std::nullopt;
  }
  NmeaVtg vtg;
  if (auto cog = parse_double(fields[1])) {
    vtg.course_over_ground_true_deg = *cog;
  }
  // Speed in km/h is field 7 (per NMEA 0183 VTG). Convert to m/s.
  if (auto sog_kmh = parse_double(fields[7])) {
    vtg.speed_over_ground_mps = *sog_kmh / 3.6;
  } else if (auto sog_kts = parse_double(fields[5])) {
    vtg.speed_over_ground_mps = *sog_kts * kKnotsToMps;
  }
  return vtg;
}

}  // namespace um982_driver
