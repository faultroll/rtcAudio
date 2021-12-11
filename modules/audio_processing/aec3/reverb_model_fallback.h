/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_REVERB_MODEL_FALLBACK_H_
#define MODULES_AUDIO_PROCESSING_AEC3_REVERB_MODEL_FALLBACK_H_

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"

namespace webrtc {

// The ReverbModelFallback class describes an exponential reverberant model.
// This model is expected to be applied over the echo power spectrum that
// is estimated by the linear filter.

class ReverbModelFallback {
 public:
  explicit ReverbModelFallback(size_t length_blocks);
  ~ReverbModelFallback();

  // Resets the state
  void Reset();

  // Adds the estimated unmodelled echo power to the residual echo power
  // estimate.
  void AddEchoReverb(RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ S2,
                     size_t delay,
                     float reverb_decay_factor,
                     RTC_VIEW(float) /* kFftLengthBy2Plus1 */R2);

  // Returns the current power spectrum reverberation contributions.
  RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ GetPowerSpectrum() const {
    return RTC_MAKE_VIEW(const float)(R2_reverb_);
  }

 private:
  float R2_reverb_[kFftLengthBy2Plus1];
  RTC_VIEW(float) R2_reverb_view_ = RTC_MAKE_VIEW(float)(R2_reverb_);
  int S2_old_index_ = 0;
  std::vector<std::array<float, kFftLengthBy2Plus1>> S2_old_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_REVERB_MODEL_FALLBACK_H_
