/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/rnn_vad/fft_util.h"

#include <cmath>

#include "rtc_base/checks.h"

namespace webrtc {
namespace rnn_vad {
namespace {

constexpr size_t kHalfFrameSize = kFrameSize20ms24kHz / 2;

// Computes the first half of the Vorbis window.
void ComputeHalfVorbisWindow(RTC_VIEW(float) /* kHalfFrameSize */ half_window) {
  // float half_window[kHalfFrameSize];
  for (size_t i = 0; i < kHalfFrameSize; ++i) {
    half_window[i] =
        std::sin(0.5 * kPi * std::sin(0.5 * kPi * (i + 0.5) / kHalfFrameSize) *
                 std::sin(0.5 * kPi * (i + 0.5) / kHalfFrameSize));
  }
  // return half_window;
}

}  // namespace

BandAnalysisFft::BandAnalysisFft()
    : half_window_(),
      half_window_view_(RTC_MAKE_VIEW(float)(half_window_)),
      input_buf_view_(RTC_MAKE_VIEW(std::complex<float>)(input_buf_)),
      fft_(static_cast<int>(input_buf_view_.size())) {
        ComputeHalfVorbisWindow(half_window_view_);
    }

BandAnalysisFft::~BandAnalysisFft() {}

void BandAnalysisFft::ForwardFft(RTC_VIEW(const float) samples,
                                 RTC_VIEW(std::complex<float>) dst) {
  RTC_DCHECK_EQ(input_buf_view_.size(), samples.size());
  RTC_DCHECK_EQ(samples.size(), dst.size());
  // Apply windowing.
  RTC_DCHECK_EQ(input_buf_view_.size(), 2 * half_window_view_.size());
  for (size_t i = 0; i < input_buf_view_.size() / 2; ++i) {
    input_buf_view_[i].real(samples[i] * half_window_view_[i]);
    size_t j = kFrameSize20ms24kHz - i - 1;
    input_buf_view_[j].real(samples[j] * half_window_view_[i]);
  }
  fft_.ForwardFft(kFrameSize20ms24kHz, input_buf_view_.data(), kFrameSize20ms24kHz,
                  dst.data());
}

}  // namespace rnn_vad
}  // namespace webrtc
