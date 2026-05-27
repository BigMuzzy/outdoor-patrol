// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "um982_driver/frame_splitter.hpp"

using um982_driver::FrameSplitter;
using um982_driver::Sentence;
using um982_driver::SentenceKind;

static void feed(FrameSplitter & sp, const std::string & s, std::vector<Sentence> & out)
{
  sp.push(reinterpret_cast<const uint8_t *>(s.data()), s.size(), out);
}

TEST(FrameSplitter, EmitsTwoSentencesFromCombinedStream)
{
  FrameSplitter sp;
  std::vector<Sentence> out;
  feed(sp,
    "$GNGGA,012345.00,3723.2475,N,12158.3416,W,4,12,0.7,18.20,M,-25.6,M,1.2,0102*7B\r\n"
    "#UNIHEADINGA,header,...;1,2,3,*ABCDEF12\r\n",
    out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].kind, SentenceKind::kNmea);
  EXPECT_EQ(out[0].text.front(), '$');
  EXPECT_EQ(out[1].kind, SentenceKind::kUnicoreAscii);
  EXPECT_EQ(out[1].text.front(), '#');
}

TEST(FrameSplitter, SkipsLeadingBinaryGarbage)
{
  FrameSplitter sp;
  std::vector<Sentence> out;
  uint8_t junk[] = {0xD3, 0x00, 0x13, 0xFF, 0x01, 0x02};
  sp.push(junk, sizeof(junk), out);
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(sp.bytes_discarded(), sizeof(junk));
  feed(sp, "$GNRMC,012345.00,A,3723.2475,N,12158.3416,W,10.0,123.4,010125,,,A*72\r\n", out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, SentenceKind::kNmea);
}

TEST(FrameSplitter, HandlesByteAtATime)
{
  FrameSplitter sp;
  std::vector<Sentence> out;
  std::string s = "$GNGGA,A,B,C*00\r\n";
  for (char c : s) {
    uint8_t b = static_cast<uint8_t>(c);
    sp.push(&b, 1, out);
  }
  ASSERT_EQ(out.size(), 1u);
}

TEST(FrameSplitter, ResynchronisesOnUnterminatedSentence)
{
  FrameSplitter sp;
  std::vector<Sentence> out;
  // Open a sentence then start a new one without terminator -> first is dropped.
  feed(sp, "$BADSENTENCE,no_terminator", out);
  EXPECT_TRUE(out.empty());
  feed(sp, "$GNGGA,A*00\r\n", out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(out[0].text.find("GNGGA"), std::string::npos);
}

TEST(FrameSplitter, OverflowProtection)
{
  FrameSplitter sp;
  std::vector<Sentence> out;
  std::string big(FrameSplitter::kMaxSentenceLen + 16, 'X');
  std::string s = "$" + big;
  feed(sp, s, out);
  EXPECT_TRUE(out.empty());
  EXPECT_GE(sp.overflow_count(), 1u);
  // Should still recover.
  feed(sp, "$GNGGA,A*00\r\n", out);
  ASSERT_EQ(out.size(), 1u);
}
