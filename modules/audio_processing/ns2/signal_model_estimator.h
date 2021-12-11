/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_SIGNAL_MODEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_NS_SIGNAL_MODEL_ESTIMATOR_H_

// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/histograms.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/prior_signal_model.h"
#include "modules/audio_processing/ns2/prior_signal_model_estimator.h"
#include "modules/audio_processing/ns2/signal_model.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

class SignalModelEstimator {
 public:
  SignalModelEstimator();

  // Compute signal normalization during the initial startup phase.
  void AdjustNormalization(int32_t num_analyzed_frames, float signal_energy);

  void Update(
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ prior_snr,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ post_snr,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ conservative_noise_spectrum,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum,
      float signal_spectral_sum,
      float signal_energy);

  const PriorSignalModel& get_prior_model() const {
    return prior_model_estimator_.get_prior_model();
  }
  const SignalModel& get_model() { return features_; }

 private:
  float diff_normalization_ = 0.f;
  float signal_energy_sum_ = 0.f;
  Histograms histograms_;
  int histogram_analysis_counter_ = 500;
  PriorSignalModelEstimator prior_model_estimator_;
  SignalModel features_;

  RTC_DISALLOW_COPY_AND_ASSIGN(SignalModelEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_SIGNAL_MODEL_ESTIMATOR_H_
