/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_WIENER_FILTER_H_
#define MODULES_AUDIO_PROCESSING_NS_WIENER_FILTER_H_

// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/suppression_params.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Estimates a Wiener-filter based frequency domain noise reduction filter.
class WienerFilter {
 public:
  explicit WienerFilter(const SuppressionParams& suppression_params);

  // Updates the filter estimate.
  void Update(
      int32_t num_analyzed_frames,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ noise_spectrum,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ prev_noise_spectrum,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ parametric_noise_spectrum,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum);

  // Compute an overall gain scaling factor.
  float ComputeOverallScalingFactor(int32_t num_analyzed_frames,
                                    float prior_speech_probability,
                                    float energy_before_filtering,
                                    float energy_after_filtering) const;

  // Returns the filter.
  RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ get_filter() const {
    return RTC_MAKE_VIEW(const float)(filter_);
  }

 private:
  const SuppressionParams& suppression_params_;
  float spectrum_prev_process_[kFftSizeBy2Plus1];
  float initial_spectral_estimate_[kFftSizeBy2Plus1];
  float filter_[kFftSizeBy2Plus1];

  RTC_DISALLOW_COPY_AND_ASSIGN(WienerFilter);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_WIENER_FILTER_H_
