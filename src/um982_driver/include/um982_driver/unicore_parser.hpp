// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__UNICORE_PARSER_HPP_
#define UM982_DRIVER__UNICORE_PARSER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace um982_driver
{

/// UM982 `$KSXT` proprietary sentence (NMEA-style, XOR checksum).
///
/// Field layout (per Unicore Reference Commands Manual):
///   1  UTC time (HHMMSS.SS)
///   2  Longitude (deg, signed decimal)
///   3  Latitude (deg, signed decimal)
///   4  Height (m, ellipsoidal)
///   5  Heading (deg, dual-antenna, master->slave, true north, CW)
///   6  Pitch (deg)
///   7  Track over ground (deg)
///   8  Speed over ground (km/h)
///   9  Roll (deg)             -- some firmwares; may be empty
///   10 Position quality (0=none, 1=single, 2=DGPS, 4=fixed RTK,
///                       5=float RTK, 6=INS, etc.)
///   11 Heading quality (0=none, 4=fixed, 5=float)
///   12 Satellites used in position
///   13 Satellites used in heading solution
///   14 East velocity (km/h)
///   15 North velocity (km/h)
///   16 Up velocity (km/h)
///
/// Field count varies between firmware versions; the parser accepts any
/// sentence with at least the first 11 fields populated.
struct KsxtSentence
{
  std::string utc;
  double longitude_deg{0.0};
  double latitude_deg{0.0};
  double height_m{0.0};
  std::optional<double> heading_deg;
  std::optional<double> pitch_deg;
  std::optional<double> track_deg;
  std::optional<double> speed_mps;          ///< converted from km/h
  std::optional<double> roll_deg;
  uint8_t position_quality{0};
  uint8_t heading_quality{0};
  std::optional<uint16_t> num_satellites_position;
  std::optional<uint16_t> num_satellites_heading;
  std::optional<double> velocity_east_mps;  ///< converted from km/h
  std::optional<double> velocity_north_mps;
  std::optional<double> velocity_up_mps;
};

/// Parse a `$KSXT,...*HH` sentence. Caller should have verified the XOR
/// checksum (e.g. via verify_nmea_checksum()).
std::optional<KsxtSentence> parse_ksxt(std::string_view sentence);

}  // namespace um982_driver

#endif  // UM982_DRIVER__UNICORE_PARSER_HPP_
