/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_QUANTILE_NOISE_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_NS_QUANTILE_NOISE_ESTIMATOR_H_

#include <math.h>
// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

constexpr int kSimult = 3;

// For quantile noise estimation.
class QuantileNoiseEstimator {
 public:
  QuantileNoiseEstimator();

  // Estimate noise.
  void Estimate(RTC_VIEW(const float) /* kFftSizeBy2Plus1 */ signal_spectrum,
                RTC_VIEW(float) /* kFftSizeBy2Plus1 */ noise_spectrum);

 private:
  float density_[kSimult * kFftSizeBy2Plus1];
  float log_quantile_[kSimult * kFftSizeBy2Plus1];
  float quantile_[kFftSizeBy2Plus1];
  int counter_[kSimult];
  int num_updates_ = 1;

  RTC_DISALLOW_COPY_AND_ASSIGN(QuantileNoiseEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_QUANTILE_NOISE_ESTIMATOR_H_
