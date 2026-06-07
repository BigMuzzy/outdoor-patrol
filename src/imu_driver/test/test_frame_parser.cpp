// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "imu_driver/frame_parser.hpp"
#include "imu_driver/messages.hpp"

using imu_driver::Frame;
using imu_driver::FrameParser;

namespace
{

// Build a valid data frame with the given data_id and payload.
std::vector<uint8_t> make_frame(
  uint8_t msg_type, uint8_t data_id,
  const std::vector<uint8_t> & payload)
{
  const uint16_t length_field = static_cast<uint16_t>(payload.size() + 6);
  std::vector<uint8_t> f = {
    0xAA, 0x55, msg_type, data_id,
    static_cast<uint8_t>(length_field & 0xFF),
    static_cast<uint8_t>((length_field >> 8) & 0xFF),
  };
  f.insert(f.end(), payload.begin(), payload.end());
  uint16_t sum = 0;
  for (size_t i = 2; i < f.size(); ++i) {
    sum = static_cast<uint16_t>(sum + f[i]);
  }
  f.push_back(static_cast<uint8_t>(sum & 0xFF));
  f.push_back(static_cast<uint8_t>((sum >> 8) & 0xFF));
  return f;
}

}  // namespace

TEST(FrameParser, ParsesSingleWholeFrame)
{
  FrameParser p;
  std::vector<Frame> out;
  auto f = make_frame(1, 0x81, {1, 2, 3, 4});
  p.push(f.data(), f.size(), out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].msg_type, 1u);
  EXPECT_EQ(out[0].data_id, 0x81u);
  EXPECT_EQ(out[0].payload, (std::vector<uint8_t>{1, 2, 3, 4}));
  EXPECT_EQ(p.frames_parsed(), 1u);
  EXPECT_EQ(p.checksum_errors(), 0u);
}

TEST(FrameParser, ReassemblesFrameSplitAcrossPushes)
{
  FrameParser p;
  std::vector<Frame> out;
  auto f = make_frame(1, 0x12, {9, 8, 7, 6, 5});
  // Feed one byte at a time.
  for (uint8_t b : f) {
    p.push(&b, 1, out);
  }
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].data_id, 0x12u);
  EXPECT_EQ(out[0].payload.size(), 5u);
}

TEST(FrameParser, SkipsLeadingGarbageThenParses)
{
  FrameParser p;
  std::vector<Frame> out;
  std::vector<uint8_t> stream = {0x00, 0xAA, 0x11, 0x22, 0xAA};  // false syncs
  auto f = make_frame(1, 0x81, {42});
  stream.insert(stream.end(), f.begin(), f.end());

  p.push(stream.data(), stream.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].payload, (std::vector<uint8_t>{42}));
  EXPECT_GT(p.bytes_discarded(), 0u);
}

TEST(FrameParser, RejectsCorruptedChecksum)
{
  FrameParser p;
  std::vector<Frame> out;
  auto f = make_frame(1, 0x81, {1, 2, 3, 4});
  f[f.size() - 1] ^= 0xFF;  // corrupt the checksum high byte

  p.push(f.data(), f.size(), out);
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(p.checksum_errors(), 1u);
  EXPECT_EQ(p.frames_parsed(), 0u);
}

TEST(FrameParser, ParsesBackToBackFrames)
{
  FrameParser p;
  std::vector<Frame> out;
  auto a = make_frame(1, 0x81, {1});
  auto b = make_frame(1, 0x1A, {2, 3, 4, 5});
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), a.begin(), a.end());
  stream.insert(stream.end(), b.begin(), b.end());

  p.push(stream.data(), stream.size(), out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].data_id, 0x81u);
  EXPECT_EQ(out[1].data_id, 0x1Au);
  EXPECT_EQ(p.frames_parsed(), 2u);
}

TEST(FrameParser, WaitsForIncompleteFrame)
{
  FrameParser p;
  std::vector<Frame> out;
  auto f = make_frame(1, 0x81, {1, 2, 3, 4, 5, 6});
  // Feed all but the last two bytes; nothing should be emitted yet.
  p.push(f.data(), f.size() - 2, out);
  EXPECT_TRUE(out.empty());
  // Feed the rest.
  p.push(f.data() + f.size() - 2, 2, out);
  EXPECT_EQ(out.size(), 1u);
}

TEST(FrameParser, RecoversAfterCorruptedFrame)
{
  FrameParser p;
  std::vector<Frame> out;
  auto bad = make_frame(1, 0x81, {1, 2, 3, 4});
  bad[bad.size() - 1] ^= 0xFF;
  auto good = make_frame(1, 0x81, {7, 7, 7, 7});
  std::vector<uint8_t> stream;
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());

  p.push(stream.data(), stream.size(), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].payload, (std::vector<uint8_t>{7, 7, 7, 7}));
  EXPECT_GE(p.checksum_errors(), 1u);
}
