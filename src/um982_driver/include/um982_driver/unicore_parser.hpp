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
///   10 Position quality (KSXT: 0=invalid, 1=single, 2=RTK float, 3=RTK fixed)
///   11 Heading quality  (KSXT: 0=invalid, 1=single, 2=RTK float, 3=RTK fixed)
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

/// Verify the 32-bit CRC of a Unicore ASCII message (`#...*HHHHHHHH`).
///
/// Unicore `#`-framed logs (e.g. `#UNIHEADINGA`, `#VERSIONA`) terminate with a
/// 32-bit CRC, not the 8-bit XOR used by NMEA `$` sentences. The algorithm is
/// the one published in the Unicore Reference Commands Manual, Appendix 1
/// ("32-bit CRC"): reflected CRC-32 (polynomial 0xEDB88320) with an initial
/// value of 0 and **no** final XOR. The CRC covers the bytes between the
/// leading `#` and the `*` (both excluded). A trailing CR/LF is tolerated.
///
/// Note: this is the receive-path message CRC. It differs from the zlib/PKZIP
/// CRC-32 utility in command_builder.cpp; UM982 *input* commands themselves are
/// sent without any checksum.
bool verify_unicore_checksum(std::string_view sentence);

}  // namespace um982_driver

#endif  // UM982_DRIVER__UNICORE_PARSER_HPP_
