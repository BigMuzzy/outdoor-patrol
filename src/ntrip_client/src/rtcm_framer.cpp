// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "ntrip_client/rtcm_framer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace ntrip_client
{

namespace
{

/// Precomputed CRC24Q table (polynomial 0x1864CFB).
std::array<uint32_t, 256> build_crc24q_table()
{
  std::array<uint32_t, 256> table{};
  constexpr uint32_t poly = 0x1864CFBu;
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if (crc & 0x1000000u) {
        crc ^= poly;
      }
    }
    table[i] = crc & 0xFFFFFFu;
  }
  return table;
}

const std::array<uint32_t, 256> & crc24q_table()
{
  static const std::array<uint32_t, 256> table = build_crc24q_table();
  return table;
}

}  // namespace

uint32_t crc24q(const uint8_t * data, std::size_t len)
{
  const auto & table = crc24q_table();
  uint32_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    const uint8_t idx = static_cast<uint8_t>((crc >> 16) ^ data[i]);
    crc = ((crc << 8) ^ table[idx]) & 0xFFFFFFu;
  }
  return crc;
}

RtcmFramer::RtcmFramer(FrameHandler on_frame)
: on_frame_(std::move(on_frame))
{
  buffer_.reserve(kRtcm3MaxFrameSize * 2);
}

void RtcmFramer::push(const uint8_t * data, std::size_t len)
{
  if (len == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + len);
  try_extract_frames();
}

void RtcmFramer::reset()
{
  bytes_discarded_ += buffer_.size();
  buffer_.clear();
}

void RtcmFramer::try_extract_frames()
{
  std::size_t cursor = 0;
  while (cursor < buffer_.size()) {
    if (buffer_[cursor] != kRtcm3Preamble) {
      ++cursor;
      ++bytes_discarded_;
      continue;
    }

    if (buffer_.size() - cursor < kRtcm3HeaderSize) {
      break;
    }

    // The upper 6 bits of byte 1 are reserved and must be zero per RTCM3.
    if ((buffer_[cursor + 1] & 0xFCu) != 0) {
      ++cursor;
      ++bytes_discarded_;
      continue;
    }
    const std::size_t length =
      (static_cast<std::size_t>(buffer_[cursor + 1] & 0x03u) << 8) |
      static_cast<std::size_t>(buffer_[cursor + 2]);
    const std::size_t frame_size = kRtcm3HeaderSize + length + kRtcm3CrcSize;

    if (buffer_.size() - cursor < frame_size) {
      break;
    }

    const uint32_t computed = crc24q(&buffer_[cursor], kRtcm3HeaderSize + length);
    const std::size_t crc_off = cursor + kRtcm3HeaderSize + length;
    const uint32_t received =
      (static_cast<uint32_t>(buffer_[crc_off]) << 16) |
      (static_cast<uint32_t>(buffer_[crc_off + 1]) << 8) |
      static_cast<uint32_t>(buffer_[crc_off + 2]);

    if (computed != received) {
      ++bad_crc_count_;
      ++cursor;
      ++bytes_discarded_;
      continue;
    }

    if (on_frame_) {
      std::vector<uint8_t> frame(
        buffer_.begin() + static_cast<std::ptrdiff_t>(cursor),
        buffer_.begin() + static_cast<std::ptrdiff_t>(cursor + frame_size));
      on_frame_(frame);
    }
    ++frames_emitted_;
    cursor += frame_size;
  }

  if (cursor > 0) {
    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + static_cast<std::ptrdiff_t>(cursor));
  }
}

}  // namespace ntrip_client
