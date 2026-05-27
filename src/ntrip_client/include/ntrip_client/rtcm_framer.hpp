// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef NTRIP_CLIENT__RTCM_FRAMER_HPP_
#define NTRIP_CLIENT__RTCM_FRAMER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace ntrip_client
{

/// RTCM3 frame structure:
///
///   byte 0       : preamble 0xD3
///   byte 1       : 6 reserved bits (must be 0) + top 2 bits of length
///   byte 2       : low 8 bits of length (`length` = 0..1023)
///   bytes 3..    : `length` payload bytes
///   last 3 bytes : CRC24Q over preamble + length + payload
///
/// Total frame size = `length + 6`.
constexpr uint8_t kRtcm3Preamble = 0xD3;
constexpr std::size_t kRtcm3HeaderSize = 3;
constexpr std::size_t kRtcm3CrcSize = 3;
constexpr std::size_t kRtcm3MaxPayload = 1023;
constexpr std::size_t kRtcm3MaxFrameSize =
  kRtcm3HeaderSize + kRtcm3MaxPayload + kRtcm3CrcSize;

/// Streaming RTCM3 frame splitter.
///
/// Feed bytes via `push()`; the framer emits one validated RTCM3 frame
/// (including the 3-byte header and 3-byte CRC24Q trailer) per callback.
class RtcmFramer
{
public:
  using FrameHandler = std::function<void (const std::vector<uint8_t> & frame)>;

  explicit RtcmFramer(FrameHandler on_frame);

  /// Push raw bytes from a socket / serial port. Calls the handler once
  /// per validated frame.
  void push(const uint8_t * data, std::size_t len);

  /// Reset internal state (e.g. after a disconnect).
  void reset();

  /// Diagnostic counters.
  std::size_t frames_emitted() const noexcept {return frames_emitted_;}
  std::size_t bad_crc_count() const noexcept {return bad_crc_count_;}
  std::size_t bytes_discarded() const noexcept {return bytes_discarded_;}

private:
  /// Drain the front of `buffer_`, emitting any complete & valid frames.
  void try_extract_frames();

  FrameHandler on_frame_;
  std::vector<uint8_t> buffer_;
  std::size_t frames_emitted_{0};
  std::size_t bad_crc_count_{0};
  std::size_t bytes_discarded_{0};
};

/// CRC24Q (RTCM-3 polynomial 0x1864CFB) over `len` bytes starting at `data`.
/// Initial value 0x000000, no reflection, no final XOR.
uint32_t crc24q(const uint8_t * data, std::size_t len);

}  // namespace ntrip_client

#endif  // NTRIP_CLIENT__RTCM_FRAMER_HPP_
