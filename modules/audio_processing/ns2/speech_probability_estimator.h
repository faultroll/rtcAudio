/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_SPEECH_PROBABILITY_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_NS_SPEECH_PROBABILITY_ESTIMATOR_H_

// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/signal_model_estimator.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Class for estimating the probability of speech.
class SpeechProbabilityEstimator {
 public:
  SpeechProbabilityEstimator();

  // Compute speech probability.
  void Update(
      int32_t num_analyzed_frames,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ prior_snr,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ post_snr,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ conservative_noise_spectrum,
      RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum,
      float signal_spectral_sum,
      float signal_energy);

  float get_prior_probability() const { return prior_speech_prob_; }
  RTC_VIEW(const float) get_probability() { return RTC_MAKE_VIEW(const float)(speech_probability_); }

 private:
  SignalModelEstimator signal_model_estimator_;
  float prior_speech_prob_ = .5f;
  float speech_probability_[kFftSizeBy2Plus1];

  RTC_DISALLOW_COPY_AND_ASSIGN(SpeechProbabilityEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_SPEECH_PROBABILITY_ESTIMATOR_H_
