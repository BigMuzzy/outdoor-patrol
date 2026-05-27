// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
#include "um982_driver/frame_splitter.hpp"

#include <cstdint>
#include <vector>

namespace um982_driver
{

void FrameSplitter::reset()
{
  state_ = State::kIdle;
  current_.clear();
}

void FrameSplitter::push(const uint8_t * data, size_t len, std::vector<Sentence> & out)
{
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    switch (state_) {
      case State::kIdle:
        if (b == '$') {
          state_ = State::kInNmea;
          current_.clear();
          current_.push_back(static_cast<char>(b));
        } else if (b == '#') {
          state_ = State::kInUnicore;
          current_.clear();
          current_.push_back(static_cast<char>(b));
        } else {
          ++bytes_discarded_;
        }
        break;
      case State::kInNmea:
      case State::kInUnicore:
        if (b == '\r' || b == '\n') {
          // End of sentence. Emit only if we got past just the sync byte.
          if (current_.size() >= 2) {
            Sentence s;
            s.kind = (state_ == State::kInNmea) ?
              SentenceKind::kNmea : SentenceKind::kUnicoreAscii;
            s.text = std::move(current_);
            out.push_back(std::move(s));
            ++sentences_emitted_;
          } else {
            bytes_discarded_ += current_.size();
          }
          current_.clear();
          state_ = State::kIdle;
        } else if (b == '$' || b == '#') {
          // Lost framing — restart on new sync byte.
          bytes_discarded_ += current_.size();
          current_.clear();
          current_.push_back(static_cast<char>(b));
          state_ = (b == '$') ? State::kInNmea : State::kInUnicore;
        } else {
          current_.push_back(static_cast<char>(b));
          if (current_.size() > kMaxSentenceLen) {
            ++overflow_count_;
            bytes_discarded_ += current_.size();
            current_.clear();
            state_ = State::kIdle;
          }
        }
        break;
    }
  }
}

}  // namespace um982_driver
