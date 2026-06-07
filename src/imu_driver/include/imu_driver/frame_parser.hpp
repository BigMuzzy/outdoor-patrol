// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef IMU_DRIVER__FRAME_PARSER_HPP_
#define IMU_DRIVER__FRAME_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace imu_driver
{

/// A decoded Inertial Labs binary frame (Table 6.2 of the KERNEL ICD).
///
/// Wire layout (little-endian, "LSB first"):
///   [0]=0xAA [1]=0x55 [2]=msg_type [3]=data_id [4..5]=length
///   [6..6+payload_len-1]=payload [..]=checksum(2)
/// where `length` is the byte count excluding the 2-byte header
/// (length = payload_len + 6) and the 16-bit checksum is the arithmetic
/// sum of every byte from index 2 through the last payload byte.
struct Frame
{
  uint8_t msg_type;            ///< 0 = command, 1 = data (device outputs = 1).
  uint8_t data_id;             ///< Echo of the command code that started the stream.
  std::vector<uint8_t> payload;
};

/// Incremental, resynchronising parser for the Inertial Labs binary framing.
///
/// Feed arbitrary byte chunks via push(); completed, checksum-valid frames are
/// appended to the caller's vector. Bytes that are not part of a valid frame
/// (line noise, partial frames after a bad checksum, interleaved unknown data)
/// are discarded and counted, and the parser resynchronises on the next
/// `0xAA 0x55` header.
class FrameParser
{
public:
  /// Reject frames claiming a payload larger than this (resync guard).
  static constexpr size_t kMaxPayloadLen = 4096;

  /// Minimum value of the on-wire length field: msg_type + data_id +
  /// length(2) + checksum(2), i.e. a zero-length payload.
  static constexpr uint16_t kMinLengthField = 6;

  /// Push `len` bytes and append any completed frames to `out`.
  void push(const uint8_t * data, size_t len, std::vector<Frame> & out);

  /// Drop any buffered partial frame and reset to the hunting state.
  void reset();

  size_t frames_parsed() const noexcept {return frames_parsed_;}
  size_t checksum_errors() const noexcept {return checksum_errors_;}
  size_t bytes_discarded() const noexcept {return bytes_discarded_;}
  size_t resyncs() const noexcept {return resyncs_;}

private:
  /// Drop the leading byte of the working buffer, counting it as discarded.
  void drop_front();

  std::vector<uint8_t> buf_;
  size_t frames_parsed_{0};
  size_t checksum_errors_{0};
  size_t bytes_discarded_{0};
  size_t resyncs_{0};
};

}  // namespace imu_driver

#endif  // IMU_DRIVER__FRAME_PARSER_HPP_
