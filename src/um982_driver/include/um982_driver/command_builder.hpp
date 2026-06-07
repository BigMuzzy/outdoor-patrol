// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__COMMAND_BUILDER_HPP_
#define UM982_DRIVER__COMMAND_BUILDER_HPP_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace um982_driver
{

/// CRC32 used by Unicore for config-command checksums (zlib CRC32:
/// polynomial 0xEDB88320, reflected, init 0xFFFFFFFF, xor-out 0xFFFFFFFF).
uint32_t unicore_crc32(const uint8_t * data, size_t len);

/// Append the Unicore CRC32 checksum to a command body and a trailing
/// CR/LF. The body must not contain the leading marker or `*`. Result is
/// `body*XXXXXXXX\r\n`.
std::string format_unicore_command(std::string_view body);

/// Build the rover-mode command.
/// `dynamics` may be `""`, `"UAV"`, `"SURVEY"`, `"SURVEY MOW"`, `"AUTOMOTIVE"`.
std::string build_mode_rover(std::string_view dynamics);

/// Build a fixed base-station command using ECEF/LLA.
std::string build_mode_base_fixed(double lat_deg, double lon_deg, double height_m);

/// Build a survey-in base command. `dist_m` may be 0 to disable the
/// position-stability requirement.
std::string build_mode_base_survey(double averaging_s, double dist_m);

/// Build the dual-antenna heading mode command.
/// `mode` is one of `""`, `"FIXLENGTH"`, `"VARIABLELENGTH"`, `"STATIC"`,
/// `"LOWDYNAMIC"`.
std::string build_mode_heading2(std::string_view mode);

/// Enable an RTCM output on a COM port at a given period (seconds).
/// e.g. build_rtcm_output(1074, "com2", 1.0).
std::string build_rtcm_output(int rtcm_id, std::string_view com, double period_s);

/// Enable an NMEA / Unicore output message at a given period (seconds).
/// Uses the Unicore shorthand "<MESSAGE> [PORT] <rate>"; an empty `com`
/// targets the current port. NMEA names are GP-prefixed.
/// e.g. build_log("GPGGA", "", 0.2) -> "GPGGA 0.2".
std::string build_log(std::string_view message, std::string_view com, double period_s);

/// Unlog all configured messages.
std::string build_unlogall();

/// Save current config to NVM. Use sparingly (flash wear).
std::string build_saveconfig();

/// Apply antenna delta (ANTENNADELTAHEN) in metres.
std::string build_antenna_delta_hen(double height, double east, double north);

}  // namespace um982_driver

#endif  // UM982_DRIVER__COMMAND_BUILDER_HPP_
