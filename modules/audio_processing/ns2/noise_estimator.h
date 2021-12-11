/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_NOISE_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_NS_NOISE_ESTIMATOR_H_

// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/quantile_noise_estimator.h"
#include "modules/audio_processing/ns2/suppression_params.h"

namespace webrtc {

// Class for estimating the spectral characteristics of the noise in an incoming
// signal.
class NoiseEstimator {
 public:
  explicit NoiseEstimator(const SuppressionParams& suppression_params);

  // Prepare the estimator for analysis of a new frame.
  void PrepareAnalysis();

  // Performs the first step of the estimator update.
  void PreUpdate(int32_t num_analyzed_frames,
                 RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum,
                 float signal_spectral_sum);

  // Performs the second step of the estimator update.
  void PostUpdate(
      RTC_VIEW(const float) speech_probability,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum);

  // Returns the noise spectral estimate.
  RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ get_noise_spectrum() 
      const {
    return RTC_MAKE_VIEW(const float)(noise_spectrum_);
  }

  // Returns the noise from the previous frame.
  RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ get_prev_noise_spectrum()
      const {
    return RTC_MAKE_VIEW(const float)(prev_noise_spectrum_);
  }

  // Returns a noise spectral estimate based on white and pink noise parameters.
 RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ get_parametric_noise_spectrum()
      const {
    return RTC_MAKE_VIEW(const float)(parametric_noise_spectrum_);
  }
  RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ get_conservative_noise_spectrum() 
      const {
    return RTC_MAKE_VIEW(const float)(conservative_noise_spectrum_);
  }

 private:
  const SuppressionParams& suppression_params_;
  float white_noise_level_ = 0.f;
  float pink_noise_numerator_ = 0.f;
  float pink_noise_exp_ = 0.f;
  float prev_noise_spectrum_[kFftSizeBy2Plus1];
  float conservative_noise_spectrum_[kFftSizeBy2Plus1];
  float parametric_noise_spectrum_[kFftSizeBy2Plus1];
  float noise_spectrum_[kFftSizeBy2Plus1];
  QuantileNoiseEstimator quantile_noise_estimator_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_NOISE_ESTIMATOR_H_
