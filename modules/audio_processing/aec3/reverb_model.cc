/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/aec3/reverb_model.h"

#include <stddef.h>

#include <algorithm>
// #include <functional>

#include "rtc_base/view.h"

namespace webrtc {

ReverbModel::ReverbModel() {
  Reset();
}

ReverbModel::~ReverbModel() {}

void ReverbModel::Reset() {
  reverb_view_.fill(0.);
}

void ReverbModel::AddReverbNoFreqShaping(RTC_VIEW(const float) power_spectrum,
                                         float power_spectrum_scaling,
                                         float reverb_decay,
                                         RTC_VIEW(float) reverb_power_spectrum) {
  UpdateReverbNoFreqShaping(power_spectrum, power_spectrum_scaling,
                            reverb_decay);

  // Add the power of the echo reverb to the residual echo power.
  std::transform(reverb_power_spectrum.begin(), reverb_power_spectrum.end(),
                 reverb_view_.begin(), reverb_power_spectrum.begin(),
                 std::plus<float>());
}

void ReverbModel::AddReverb(RTC_VIEW(const float) power_spectrum,
                            RTC_VIEW(const float) power_spectrum_scaling,
                            float reverb_decay,
                            RTC_VIEW(float) reverb_power_spectrum) {
  UpdateReverb(power_spectrum, power_spectrum_scaling,
               reverb_decay);

  // Add the power of the echo reverb to the residual echo power.
  std::transform(reverb_power_spectrum.begin(), reverb_power_spectrum.end(),
                 reverb_view_.begin(), reverb_power_spectrum.begin(),
                 std::plus<float>());
}

void ReverbModel::UpdateReverbNoFreqShaping(
    RTC_VIEW(const float) power_spectrum,
    float power_spectrum_scaling,
    float reverb_decay) {
  if (reverb_decay > 0) {
    // Update the estimate of the reverberant power.
    for (size_t k = 0; k < power_spectrum.size(); ++k) {
      reverb_view_[k] = (reverb_view_[k] + power_spectrum[k] * power_spectrum_scaling) *
                   reverb_decay;
    }
  }
}

void ReverbModel::UpdateReverb(
    RTC_VIEW(const float) power_spectrum,
    RTC_VIEW(const float) power_spectrum_scaling,
    float reverb_decay) {
  if (reverb_decay > 0) {
    // Update the estimate of the reverberant power.
    for (size_t k = 0; k < power_spectrum.size(); ++k) {
      reverb_view_[k] =
          (reverb_view_[k] + power_spectrum[k] * power_spectrum_scaling[k]) *
          reverb_decay;
    }
  }
}

}  // namespace webrtc
