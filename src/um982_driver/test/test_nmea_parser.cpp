// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "um982_driver/nmea_parser.hpp"

using um982_driver::parse_gga;
using um982_driver::parse_rmc;
using um982_driver::parse_vtg;
using um982_driver::parse_gst;
using um982_driver::verify_nmea_checksum;
using um982_driver::NmeaFixQuality;

// Helper: compute and append the XOR checksum to an unchecksummed body.
static std::string with_checksum(const std::string & body_without_dollar)
{
  uint8_t cs = 0;
  for (char c : body_without_dollar) {
    cs ^= static_cast<uint8_t>(c);
  }
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + body_without_dollar + buf;
}

TEST(NmeaParser, ChecksumVerifiesKnownGoodSentence)
{
  // Reference sentence from NMEA 0183 spec.
  EXPECT_TRUE(verify_nmea_checksum(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
}

TEST(NmeaParser, ChecksumRejectsTamperedSentence)
{
  EXPECT_FALSE(verify_nmea_checksum(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*46"));
}

TEST(NmeaParser, ChecksumComputerMatchesVerifier)
{
  auto s = with_checksum("GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  EXPECT_TRUE(verify_nmea_checksum(s));
}

TEST(NmeaParser, ParsesGgaRtkFixWithCorrectionAge)
{
  auto s = with_checksum(
    "GNGGA,012345.00,3723.2475,N,12158.3416,W,4,12,0.7,18.20,M,-25.6,M,1.2,0102");
  auto gga = parse_gga(s);
  ASSERT_TRUE(gga.has_value());
  EXPECT_EQ(gga->utc, "012345.00");
  EXPECT_NEAR(gga->latitude_deg, 37.0 + 23.2475 / 60.0, 1e-7);
  EXPECT_NEAR(gga->longitude_deg, -(121.0 + 58.3416 / 60.0), 1e-7);
  EXPECT_EQ(gga->quality, NmeaFixQuality::kRtkFix);
  EXPECT_EQ(gga->num_satellites, 12u);
  EXPECT_NEAR(gga->hdop, 0.7, 1e-9);
  EXPECT_NEAR(gga->altitude_m, 18.20, 1e-9);
  EXPECT_NEAR(gga->geoid_separation_m, -25.6, 1e-9);
  ASSERT_TRUE(gga->age_of_corrections_s.has_value());
  EXPECT_NEAR(*gga->age_of_corrections_s, 1.2, 1e-9);
  ASSERT_TRUE(gga->station_id.has_value());
  EXPECT_EQ(*gga->station_id, 102u);
}

TEST(NmeaParser, ParsesGgaInvalidFixWithNoPosition)
{
  auto s = with_checksum("GNGGA,000000.00,,,,,0,00,99.99,,M,,M,,");
  auto gga = parse_gga(s);
  ASSERT_TRUE(gga.has_value());
  EXPECT_EQ(gga->quality, NmeaFixQuality::kInvalid);
  EXPECT_EQ(gga->num_satellites, 0u);
  EXPECT_FALSE(gga->age_of_corrections_s.has_value());
  EXPECT_FALSE(gga->station_id.has_value());
}

TEST(NmeaParser, ParsesRmcSpeedInMetersPerSecond)
{
  auto s = with_checksum(
    "GNRMC,012345.00,A,3723.2475,N,12158.3416,W,10.0,123.4,010125,,,A");
  auto rmc = parse_rmc(s);
  ASSERT_TRUE(rmc.has_value());
  EXPECT_TRUE(rmc->valid);
  EXPECT_NEAR(rmc->speed_over_ground_mps, 10.0 * 0.5144444, 1e-5);
  ASSERT_TRUE(rmc->course_over_ground_deg.has_value());
  EXPECT_NEAR(*rmc->course_over_ground_deg, 123.4, 1e-9);
  EXPECT_EQ(rmc->date_ddmmyy, "010125");
}

TEST(NmeaParser, ParsesVtgSpeedInMetersPerSecond)
{
  auto s = with_checksum("GNVTG,123.4,T,121.0,M,10.0,N,18.52,K,A");
  auto vtg = parse_vtg(s);
  ASSERT_TRUE(vtg.has_value());
  ASSERT_TRUE(vtg->course_over_ground_true_deg.has_value());
  EXPECT_NEAR(*vtg->course_over_ground_true_deg, 123.4, 1e-9);
  EXPECT_NEAR(vtg->speed_over_ground_mps, 18.52 / 3.6, 1e-9);
}

TEST(NmeaParser, RejectsWrongSentenceType)
{
  auto s = with_checksum("GNVTG,123.4,T,121.0,M,10.0,N,18.52,K,A");
  EXPECT_FALSE(parse_gga(s).has_value());
  EXPECT_FALSE(parse_rmc(s).has_value());
}

TEST(NmeaParser, ParsesGstMeasuredStandardDeviations)
{
  // Reference example from the Unicore manual (Table 7-9 GST Data Structure):
  // $GNGST,utc,rms,smjr,smnr,orient,lat_std,lon_std,alt_std
  auto s = with_checksum("GNGST,054013.00,0.67,1.67,1.37,115.3800,1.432,1.620,3.399");
  auto gst = parse_gst(s);
  ASSERT_TRUE(gst.has_value());
  EXPECT_EQ(gst->utc, "054013.00");
  EXPECT_NEAR(gst->std_lat_m, 1.432, 1e-6);
  EXPECT_NEAR(gst->std_lon_m, 1.620, 1e-6);
  EXPECT_NEAR(gst->std_alt_m, 3.399, 1e-6);
}

TEST(NmeaParser, RejectsGstWrongTypeOrShort)
{
  EXPECT_FALSE(parse_gst(with_checksum("GNVTG,123.4,T,121.0,M,10.0,N,18.52,K,A"))
      .has_value());
  // Too few fields.
  EXPECT_FALSE(parse_gst(with_checksum("GNGST,054013.00,0.67,1.67")).has_value());
}
