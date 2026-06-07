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

// Unicore/NovAtel 32-bit message CRC, per the Reference Commands Manual,
// Appendix 1: reflected CRC-32 (poly 0xEDB88320), initial value 0, no final
// XOR. Distinct from the zlib CRC-32 in command_builder.cpp (init 0xFFFFFFFF,
// final XOR), which is only used to frame outgoing commands.
uint32_t unicore_message_crc32(const uint8_t * data, size_t len)
{
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) {
      crc = (crc & 1U) ? (0xEDB88320U ^ (crc >> 1)) : (crc >> 1);
    }
  }
  return crc;
}

int hex_nibble(char c)
{
  if (c >= '0' && c <= '9') {return c - '0';}
  if (c >= 'a' && c <= 'f') {return 10 + (c - 'a');}
  if (c >= 'A' && c <= 'F') {return 10 + (c - 'A');}
  return -1;
}

}  // namespace

bool verify_unicore_checksum(std::string_view sentence)
{
  while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
    sentence.remove_suffix(1);
  }
  // Minimum: '#' + at least one body byte + '*' + 8 hex CRC digits.
  if (sentence.size() < 11 || sentence.front() != '#') {
    return false;
  }
  auto star = sentence.rfind('*');
  if (star == std::string_view::npos || star + 9 != sentence.size()) {
    return false;
  }
  uint32_t expected = 0;
  for (size_t i = star + 1; i < sentence.size(); ++i) {
    int v = hex_nibble(sentence[i]);
    if (v < 0) {
      return false;
    }
    expected = (expected << 4) | static_cast<uint32_t>(v);
  }
  // CRC covers the bytes between the leading '#' and the '*'.
  const auto * body = reinterpret_cast<const uint8_t *>(sentence.data() + 1);
  size_t body_len = star - 1;
  return unicore_message_crc32(body, body_len) == expected;
}

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
