// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ntrip_client/rtcm_framer.hpp"

namespace
{

/// Build a syntactically valid RTCM3 frame around `payload`, computing the
/// CRC24Q trailer.
std::vector<uint8_t> make_frame(const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> f;
  const std::size_t len = payload.size();
  f.reserve(len + 6);
  f.push_back(ntrip_client::kRtcm3Preamble);
  f.push_back(static_cast<uint8_t>((len >> 8) & 0x03u));
  f.push_back(static_cast<uint8_t>(len & 0xFFu));
  f.insert(f.end(), payload.begin(), payload.end());
  const uint32_t crc = ntrip_client::crc24q(f.data(), f.size());
  f.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFu));
  f.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFu));
  f.push_back(static_cast<uint8_t>(crc & 0xFFu));
  return f;
}

}  // namespace

TEST(RtcmFramerTest, EmitsSingleValidFrame)
{
  std::vector<std::vector<uint8_t>> seen;
  ntrip_client::RtcmFramer framer(
    [&](const std::vector<uint8_t> & f) {seen.push_back(f);});

  const std::vector<uint8_t> payload{0x11, 0x22, 0x33, 0x44, 0x55};
  const auto frame = make_frame(payload);

  framer.push(frame.data(), frame.size());

  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0], frame);
  EXPECT_EQ(framer.frames_emitted(), 1u);
  EXPECT_EQ(framer.bad_crc_count(), 0u);
}

TEST(RtcmFramerTest, HandlesByteAtATimeStreaming)
{
  std::vector<std::vector<uint8_t>> seen;
  ntrip_client::RtcmFramer framer(
    [&](const std::vector<uint8_t> & f) {seen.push_back(f);});

  const std::vector<uint8_t> payload(120, 0xAA);  // > one preamble worth
  const auto frame = make_frame(payload);

  for (uint8_t b : frame) {
    framer.push(&b, 1);
  }

  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0], frame);
}

TEST(RtcmFramerTest, SkipsLeadingGarbage)
{
  std::vector<std::vector<uint8_t>> seen;
  ntrip_client::RtcmFramer framer(
    [&](const std::vector<uint8_t> & f) {seen.push_back(f);});

  const std::vector<uint8_t> payload{0x01, 0x02, 0x03};
  auto frame = make_frame(payload);
  std::vector<uint8_t> stream{0xAB, 0xCD, 0xEF, 0x00, 0xD3 /* fake */};
  // Append a bogus byte after fake preamble that fails the reserved-bits
  // check so the framer resyncs.
  stream.push_back(0xFF);
  stream.insert(stream.end(), frame.begin(), frame.end());

  framer.push(stream.data(), stream.size());

  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0], frame);
  EXPECT_GT(framer.bytes_discarded(), 0u);
}

TEST(RtcmFramerTest, RejectsCorruptedCrc)
{
  std::vector<std::vector<uint8_t>> seen;
  ntrip_client::RtcmFramer framer(
    [&](const std::vector<uint8_t> & f) {seen.push_back(f);});

  const std::vector<uint8_t> payload{0x10, 0x20, 0x30, 0x40};
  auto good = make_frame(payload);
  auto bad = good;
  bad[bad.size() - 1] ^= 0xFFu;  // corrupt last CRC byte

  framer.push(bad.data(), bad.size());
  framer.push(good.data(), good.size());

  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0], good);
  EXPECT_EQ(framer.bad_crc_count(), 1u);
}

TEST(RtcmFramerTest, EmitsTwoBackToBackFrames)
{
  std::vector<std::vector<uint8_t>> seen;
  ntrip_client::RtcmFramer framer(
    [&](const std::vector<uint8_t> & f) {seen.push_back(f);});

  const auto a = make_frame({0x01, 0x02});
  const auto b = make_frame({0x03, 0x04, 0x05, 0x06});
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), a.begin(), a.end());
  stream.insert(stream.end(), b.begin(), b.end());

  framer.push(stream.data(), stream.size());

  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0], a);
  EXPECT_EQ(seen[1], b);
}

TEST(Crc24qTest, MatchesBitwiseReferenceImplementation)
{
  // Independent bitwise reference (polynomial 0x1864CFB, initial 0, no XOR-out).
  auto bitwise = [](const uint8_t * data, std::size_t len) {
      uint32_t crc = 0;
      for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int b = 0; b < 8; ++b) {
          crc <<= 1;
          if (crc & 0x1000000u) {
            crc ^= 0x1864CFBu;
          }
        }
      }
      return crc & 0xFFFFFFu;
    };

  // Empty input -> CRC of nothing is 0.
  EXPECT_EQ(ntrip_client::crc24q(nullptr, 0), 0u);

  const std::vector<std::vector<uint8_t>> cases{
    {0xD3, 0x00, 0x00},
    {0xD3, 0x00, 0x04, 0x10, 0x20, 0x30, 0x40},
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A},
    std::vector<uint8_t>(256, 0xAAu),
  };
  for (const auto & c : cases) {
    EXPECT_EQ(ntrip_client::crc24q(c.data(), c.size()), bitwise(c.data(), c.size()));
  }
}
