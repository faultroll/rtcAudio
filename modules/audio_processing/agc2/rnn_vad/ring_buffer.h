/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RING_BUFFER_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RING_BUFFER_H_

#include <stddef.h>
#include <string.h>

// #include <array>
// #include <cstring>
// #include <type_traits>

#include "rtc_base/view.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rnn_vad {

// Ring buffer for N arrays of type T each one with size S.
template <typename T, size_t S, size_t N>
class RingBuffer {
  static_assert(S > 0, "");
  static_assert(N > 0, "");
  // static_assert(std::is_arithmetic<T>::value,
  //               "Integral or floating point required.");

 public:
  RingBuffer() : 
    tail_(0), 
    buffer_view_(RTC_MAKE_VIEW(T)(buffer_)) {}
  ~RingBuffer() {}
  // Set the ring buffer values to zero.
  void Reset() { buffer_view_.fill(0); }
  // Replace the least recently pushed array in the buffer with |new_values|.
  void Push(RTC_VIEW(const T) /* S */ new_values) {
    memcpy(buffer_view_.data() + S * tail_, new_values.data(), S * sizeof(T));
    tail_ += 1;
    if (tail_ == N)
      tail_ = 0;
  }
  // Return an array view onto the array with a given delay. A view on the last
  // and least recently push array is returned when |delay| is 0 and N - 1
  // respectively.
  RTC_VIEW(const T) /* S */ GetView(size_t delay) const {
    const int delay_int = static_cast<int>(delay);
    RTC_DCHECK_LE(0, delay_int);
    RTC_DCHECK_LT(delay_int, N);
    int offset = tail_ - 1 - delay_int;
    if (offset < 0)
      offset += N;
    return {buffer_view_.data() + S * offset, S};
  }

 private:
  int tail_;  // Index of the least recently pushed sub-array.
  T buffer_[S * N];
  RTC_VIEW(T) /* S * N */ buffer_view_;

  RTC_DISALLOW_COPY_AND_ASSIGN(RingBuffer);
};

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RING_BUFFER_H_
