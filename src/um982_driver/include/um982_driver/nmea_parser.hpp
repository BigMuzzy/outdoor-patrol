// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__NMEA_PARSER_HPP_
#define UM982_DRIVER__NMEA_PARSER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace um982_driver
{

/// NMEA-0183 fix-quality enumeration as reported in the GGA `quality` field.
enum class NmeaFixQuality : uint8_t
{
  kInvalid = 0,
  kSps = 1,
  kDgps = 2,
  kPps = 3,
  kRtkFix = 4,
  kRtkFloat = 5,
  kDeadReckoning = 6,
  kManual = 7,
  kSimulation = 8,
};

struct NmeaGga
{
  std::string utc;                          ///< raw HHMMSS.SS
  double latitude_deg{0.0};                 ///< signed decimal degrees
  double longitude_deg{0.0};                ///< signed decimal degrees
  NmeaFixQuality quality{NmeaFixQuality::kInvalid};
  uint16_t num_satellites{0};
  double hdop{0.0};
  double altitude_m{0.0};                   ///< above MSL
  double geoid_separation_m{0.0};
  std::optional<double> age_of_corrections_s;
  std::optional<uint16_t> station_id;
};

struct NmeaRmc
{
  std::string utc;
  std::string date_ddmmyy;
  bool valid{false};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double speed_over_ground_mps{0.0};        ///< converted from knots
  std::optional<double> course_over_ground_deg;
};

struct NmeaVtg
{
  std::optional<double> course_over_ground_true_deg;
  double speed_over_ground_mps{0.0};
};

/// NMEA GST — GNSS pseudorange error statistics. Carries the receiver's
/// measured 1-sigma position error estimates (metres).
struct NmeaGst
{
  std::string utc;                          ///< raw HHMMSS.SS
  double std_lat_m{0.0};                    ///< 1-sigma latitude (North) error
  double std_lon_m{0.0};                    ///< 1-sigma longitude (East) error
  double std_alt_m{0.0};                    ///< 1-sigma altitude (Up) error
};

/// XOR-checksum verifier. Accepts the full sentence including leading `$`
/// (or `!`) and the trailing `*HH` (case-insensitive). CR/LF tolerated.
bool verify_nmea_checksum(std::string_view sentence);

/// Split an NMEA sentence into its comma-separated fields. Strips the
/// leading `$`/`!` and the trailing `*HH` (and CR/LF). Returns the talker
/// + message identifier as the first element. Returns empty vector if the
/// sentence is malformed.
std::vector<std::string> split_nmea_fields(std::string_view sentence);

/// Parse a `$..GGA` sentence. Caller is expected to have verified the
/// checksum already (or pass a trusted sentence).
std::optional<NmeaGga> parse_gga(std::string_view sentence);

/// Parse a `$..RMC` sentence.
std::optional<NmeaRmc> parse_rmc(std::string_view sentence);

/// Parse a `$..VTG` sentence.
std::optional<NmeaVtg> parse_vtg(std::string_view sentence);

/// Parse a `$..GST` sentence (pseudorange error statistics). The lat/lon/alt
/// standard deviations give the receiver's measured 1-sigma position errors.
std::optional<NmeaGst> parse_gst(std::string_view sentence);

}  // namespace um982_driver

#endif  // UM982_DRIVER__NMEA_PARSER_HPP_
