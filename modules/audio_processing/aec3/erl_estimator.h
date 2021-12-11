/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_ERL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AEC3_ERL_ESTIMATOR_H_

#include <stddef.h>

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Estimates the echo return loss based on the signal spectra.
class ErlEstimator {
 public:
  explicit ErlEstimator(size_t startup_phase_length_blocks_);
  ~ErlEstimator();

  // Resets the ERL estimation.
  void Reset();

  // Updates the ERL estimate.
  void Update(const std::vector<bool>& converged_filters,
              const std::vector<std::array<float, kFftLengthBy2Plus1>>& render_spectra,
              const std::vector<std::array<float, kFftLengthBy2Plus1>>& capture_spectra);

  // Returns the most recent ERL estimate.
  RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ Erl() const { 
    return RTC_MAKE_VIEW(const float)(erl_);
  }
  float ErlTimeDomain() const { return erl_time_domain_; }

 private:
  const size_t startup_phase_length_blocks__;
  float erl_[kFftLengthBy2Plus1];
  RTC_VIEW(float) erl_view_ = RTC_MAKE_VIEW(float)(erl_);
  int hold_counters_[kFftLengthBy2Minus1];
  RTC_VIEW(int) hold_counters_view_ = RTC_MAKE_VIEW(int)(hold_counters_);
  float erl_time_domain_;
  int hold_counter_time_domain_;
  size_t blocks_since_reset_ = 0;
  RTC_DISALLOW_COPY_AND_ASSIGN(ErlEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_ERL_ESTIMATOR_H_
