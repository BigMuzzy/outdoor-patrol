// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#ifndef UM982_DRIVER__FRAME_SPLITTER_HPP_
#define UM982_DRIVER__FRAME_SPLITTER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace um982_driver
{

/// Kind of sentence emitted by FrameSplitter.
enum class SentenceKind : uint8_t
{
  kNmea,            ///< Starts with '$', ends at first CR or LF.
  kUnicoreAscii,    ///< Starts with '#', ends at first CR or LF.
};

struct Sentence
{
  SentenceKind kind;
  std::string text;   ///< Without trailing CR/LF.
};

/// Byte-stream splitter for UM982 ASCII output.
///
/// The UM982 emits two ASCII families on the same serial line:
///   - Standard NMEA sentences and proprietary `$KSXT` (line starts with `$`).
///   - Unicore ASCII messages such as `#UNIHEADINGA,...` (line starts with `#`).
/// Both terminate with `\r\n`. Binary RTCM frames may also be present on
/// the same stream; any non-ASCII bytes outside an active sentence are
/// silently discarded.
class FrameSplitter
{
public:
  /// Maximum length of a single sentence body. Bytes exceeding this from
  /// an active sentence cause the sentence to be discarded and the
  /// splitter to resynchronise on the next sync byte.
  static constexpr size_t kMaxSentenceLen = 2048;

  /// Push `len` bytes from `data` and emit any sentences that completed.
  void push(const uint8_t * data, size_t len, std::vector<Sentence> & out);

  /// Reset internal buffer. Existing in-flight sentence is discarded.
  void reset();

  size_t bytes_discarded() const noexcept {return bytes_discarded_;}
  size_t sentences_emitted() const noexcept {return sentences_emitted_;}
  size_t overflow_count() const noexcept {return overflow_count_;}

private:
  enum class State : uint8_t
  {
    kIdle,
    kInNmea,
    kInUnicore,
  };

  State state_{State::kIdle};
  std::string current_;
  size_t bytes_discarded_{0};
  size_t sentences_emitted_{0};
  size_t overflow_count_{0};
};

}  // namespace um982_driver

#endif  // UM982_DRIVER__FRAME_SPLITTER_HPP_
