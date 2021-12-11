/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_NS_FFT_H_
#define MODULES_AUDIO_PROCESSING_NS_NS_FFT_H_

#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Wrapper class providing 256 point FFT functionality.
class NrFft {
 public:
  NrFft();

  // Transforms the signal from time to frequency domain.
  void Fft(RTC_VIEW(float) /* kFftSize */ time_data,
           RTC_VIEW(float) /* kFftSize */ real,
           RTC_VIEW(float) /* kFftSize */ imag);

  // Transforms the signal from frequency to time domain.
  void Ifft(RTC_VIEW(const float) real,
            RTC_VIEW(const float) imag,
            RTC_VIEW(float) time_data);

 private:
  std::vector<size_t> bit_reversal_state_;
  std::vector<float> tables_;

  RTC_DISALLOW_COPY_AND_ASSIGN(NrFft);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_NS_FFT_H_
