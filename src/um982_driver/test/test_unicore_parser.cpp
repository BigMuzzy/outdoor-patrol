// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <string>

#include "um982_driver/nmea_parser.hpp"
#include "um982_driver/unicore_parser.hpp"

using um982_driver::parse_ksxt;
using um982_driver::verify_nmea_checksum;

static std::string with_xor(const std::string & body)
{
  uint8_t cs = 0;
  for (char c : body) {
    cs ^= static_cast<uint8_t>(c);
  }
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + body + buf;
}

TEST(UnicoreParser, KsxtParsesPositionHeadingAndQuality)
{
  // Body: KSXT,utc,lon,lat,h,heading,pitch,track,sog_kmh,roll,
  //       pos_q,hdg_q,nsats_pos,nsats_hdg,ve,vn,vu
  auto s = with_xor(
    "KSXT,012345.00,121.5031,37.3712,18.20,123.45,0.5,123.0,3.60,0.1,"
    "4,4,12,11,1.00,3.40,0.10");
  ASSERT_TRUE(verify_nmea_checksum(s));
  auto k = parse_ksxt(s);
  ASSERT_TRUE(k.has_value());
  EXPECT_EQ(k->utc, "012345.00");
  EXPECT_NEAR(k->longitude_deg, 121.5031, 1e-7);
  EXPECT_NEAR(k->latitude_deg, 37.3712, 1e-7);
  EXPECT_NEAR(k->height_m, 18.20, 1e-9);
  ASSERT_TRUE(k->heading_deg.has_value());
  EXPECT_NEAR(*k->heading_deg, 123.45, 1e-9);
  ASSERT_TRUE(k->speed_mps.has_value());
  EXPECT_NEAR(*k->speed_mps, 3.60 / 3.6, 1e-9);
  EXPECT_EQ(k->position_quality, 4u);
  EXPECT_EQ(k->heading_quality, 4u);
  ASSERT_TRUE(k->num_satellites_position.has_value());
  EXPECT_EQ(*k->num_satellites_position, 12u);
  ASSERT_TRUE(k->velocity_east_mps.has_value());
  EXPECT_NEAR(*k->velocity_east_mps, 1.00 / 3.6, 1e-9);
  ASSERT_TRUE(k->velocity_north_mps.has_value());
  EXPECT_NEAR(*k->velocity_north_mps, 3.40 / 3.6, 1e-9);
}

TEST(UnicoreParser, KsxtAcceptsShortSentenceWithoutVelocity)
{
  auto s = with_xor("KSXT,012345.00,121.5031,37.3712,18.20,,,,,,0,0");
  auto k = parse_ksxt(s);
  ASSERT_TRUE(k.has_value());
  EXPECT_FALSE(k->heading_deg.has_value());
  EXPECT_FALSE(k->velocity_east_mps.has_value());
  EXPECT_EQ(k->position_quality, 0u);
}

TEST(UnicoreParser, KsxtRejectsWrongIdentifier)
{
  auto s = with_xor("GNGGA,012345.00,3723.2475,N,12158.3416,W,4,12,0.7,18.20,M,-25.6,M,1.2,0102");
  EXPECT_FALSE(parse_ksxt(s).has_value());
}
