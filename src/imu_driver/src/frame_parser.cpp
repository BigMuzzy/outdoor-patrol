// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "imu_driver/frame_parser.hpp"

#include <vector>

namespace imu_driver
{

namespace
{
constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;
constexpr size_t kHeaderLen = 6;   ///< header(2) + msg_type + data_id + length(2)
}  // namespace

void FrameParser::drop_front()
{
  buf_.erase(buf_.begin());
  ++bytes_discarded_;
}

void FrameParser::reset()
{
  buf_.clear();
}

void FrameParser::push(const uint8_t * data, size_t len, std::vector<Frame> & out)
{
  buf_.insert(buf_.end(), data, data + len);

  while (buf_.size() >= 2) {
    // Hunt for the 0xAA 0x55 sync word.
    if (buf_[0] != kHeader0) {
      drop_front();
      continue;
    }
    if (buf_[1] != kHeader1) {
      // A lone 0xAA: drop it and keep hunting from the next byte.
      drop_front();
      continue;
    }

    // Need the full fixed header before the length is known.
    if (buf_.size() < kHeaderLen) {
      return;
    }

    const uint16_t length_field =
      static_cast<uint16_t>(buf_[4]) |
      (static_cast<uint16_t>(buf_[5]) << 8);

    // Validate the advertised length; an implausible value means we locked
    // onto a false sync word inside the payload of garbage.
    if (length_field < kMinLengthField ||
      (length_field - kMinLengthField) > static_cast<int>(kMaxPayloadLen))
    {
      drop_front();              // step past this 0xAA
      ++resyncs_;
      continue;
    }

    const size_t payload_len = static_cast<size_t>(length_field) - kMinLengthField;
    const size_t total_len = static_cast<size_t>(length_field) + 2;  // + header(2)

    if (buf_.size() < total_len) {
      return;                    // wait for the rest of the frame
    }

    // Checksum: 16-bit arithmetic sum of bytes [2 .. last payload byte].
    uint16_t sum = 0;
    const size_t checksum_span_end = total_len - 2;   // exclusive
    for (size_t i = 2; i < checksum_span_end; ++i) {
      sum = static_cast<uint16_t>(sum + buf_[i]);
    }
    const uint16_t frame_cksum =
      static_cast<uint16_t>(buf_[total_len - 2]) |
      (static_cast<uint16_t>(buf_[total_len - 1]) << 8);

    if (sum != frame_cksum) {
      drop_front();              // false sync; resync past this 0xAA
      ++checksum_errors_;
      ++resyncs_;
      continue;
    }

    Frame f;
    f.msg_type = buf_[2];
    f.data_id = buf_[3];
    f.payload.assign(buf_.begin() + kHeaderLen, buf_.begin() + kHeaderLen + payload_len);
    out.push_back(std::move(f));
    ++frames_parsed_;

    buf_.erase(buf_.begin(), buf_.begin() + total_len);
  }
}

}  // namespace imu_driver
