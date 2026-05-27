// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/unicore_parser.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "um982_driver/nmea_parser.hpp"

namespace um982_driver
{

namespace
{

std::optional<double> to_double(const std::string & s)
{
  if (s.empty()) {
    return std::nullopt;
  }
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

std::optional<int64_t> to_int(const std::string & s)
{
  if (s.empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  int64_t v = std::strtoll(s.c_str(), &end, 10);
  if (end == s.c_str()) {
    return std::nullopt;
  }
  return v;
}

}  // namespace

std::optional<KsxtSentence> parse_ksxt(std::string_view sentence)
{
  auto fields = split_nmea_fields(sentence);
  // Need at least: id + 10 fields = 11 entries.
  if (fields.size() < 11) {
    return std::nullopt;
  }
  if (fields[0] != "KSXT") {
    return std::nullopt;
  }
  KsxtSentence k;
  k.utc = fields[1];
  if (auto v = to_double(fields[2])) {k.longitude_deg = *v;}
  if (auto v = to_double(fields[3])) {k.latitude_deg = *v;}
  if (auto v = to_double(fields[4])) {k.height_m = *v;}
  if (auto v = to_double(fields[5])) {k.heading_deg = *v;}
  if (auto v = to_double(fields[6])) {k.pitch_deg = *v;}
  if (auto v = to_double(fields[7])) {k.track_deg = *v;}
  if (auto v = to_double(fields[8])) {k.speed_mps = *v / 3.6;}
  if (fields.size() > 9) {
    if (auto v = to_double(fields[9])) {k.roll_deg = *v;}
  }
  // Position / heading quality may live at indices 10/11 (no roll) or
  // 10/11 with roll present. The Unicore manual's reference layout puts
  // them at fields 10 (position) and 11 (heading) of the body — which
  // maps to fields[10] / fields[11] here.
  if (fields.size() > 10) {
    if (auto v = to_int(fields[10])) {
      k.position_quality = static_cast<uint8_t>(*v & 0xFF);
    }
  }
  if (fields.size() > 11) {
    if (auto v = to_int(fields[11])) {
      k.heading_quality = static_cast<uint8_t>(*v & 0xFF);
    }
  }
  if (fields.size() > 12) {
    if (auto v = to_int(fields[12])) {
      k.num_satellites_position = static_cast<uint16_t>(*v & 0xFFFF);
    }
  }
  if (fields.size() > 13) {
    if (auto v = to_int(fields[13])) {
      k.num_satellites_heading = static_cast<uint16_t>(*v & 0xFFFF);
    }
  }
  if (fields.size() > 14) {
    if (auto v = to_double(fields[14])) {k.velocity_east_mps = *v / 3.6;}
  }
  if (fields.size() > 15) {
    if (auto v = to_double(fields[15])) {k.velocity_north_mps = *v / 3.6;}
  }
  if (fields.size() > 16) {
    if (auto v = to_double(fields[16])) {k.velocity_up_mps = *v / 3.6;}
  }
  return k;
}

}  // namespace um982_driver
