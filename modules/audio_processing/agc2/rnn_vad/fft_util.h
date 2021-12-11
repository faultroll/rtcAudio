/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_FFT_UTIL_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_FFT_UTIL_H_

// #include <array>
#include <complex>

#include "rtc_base/view.h"
#include "modules/audio_processing/agc2/rnn_vad/common.h"
#include "third_party/rnnoise/src/kiss_fft.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rnn_vad {

// FFT implementation wrapper for the band-wise analysis step in which 20 ms
// frames at 24 kHz are analyzed in the frequency domain. The goal of this class
// are (i) making easy to switch to another FFT implementation, (ii) own the
// input buffer for the FFT and (iii) apply a windowing function before
// computing the FFT.
class BandAnalysisFft {
 public:
  BandAnalysisFft();
  ~BandAnalysisFft();
  // Applies a windowing function to |samples|, computes the real forward FFT
  // and writes the result in |dst|.
  void ForwardFft(RTC_VIEW(const float) samples,
                  RTC_VIEW(std::complex<float>) dst);

 private:
  static_assert((kFrameSize20ms24kHz & 1) == 0,
                "kFrameSize20ms24kHz must be even.");
  const float half_window_[kFrameSize20ms24kHz / 2];
  RTC_VIEW(const float) half_window_view_;
  std::complex<float> input_buf_[kFrameSize20ms24kHz];
  RTC_VIEW(std::complex<float>) input_buf_view_;
  rnnoise::KissFft fft_;
  
  RTC_DISALLOW_COPY_AND_ASSIGN(BandAnalysisFft);
};

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_FFT_UTIL_H_
