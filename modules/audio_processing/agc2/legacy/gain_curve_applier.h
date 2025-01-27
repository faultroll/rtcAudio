/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
// Same as limiter.h

#ifndef MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_
#define MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_

#include <vector>

#include "modules/audio_processing/agc2/fixed_digital_level_estimator.h"
#include "modules/audio_processing/agc2/interpolated_gain_curve.h"
#include "modules/audio_processing/include/audio_frame_view.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
class ApmDataDumper;

class GainCurveApplier {
 public:
  GainCurveApplier(size_t sample_rate_hz,
                   ApmDataDumper* apm_data_dumper,
                   std::string histogram_name_prefix);

  ~GainCurveApplier();

  void Process(AudioFrameView<float> signal);
  InterpolatedGainCurve::Stats GetGainCurveStats() const;

  // Supported rates must be
  // * supported by FixedDigitalLevelEstimator
  // * below kMaximalNumberOfSamplesPerChannel*1000/kFrameDurationMs
  //   so that samples_per_channel fit in the
  //   per_sample_scaling_factors_ array.
  void SetSampleRate(size_t sample_rate_hz);

  // Resets the internal state.
  void Reset();

 private:
  const InterpolatedGainCurve interp_gain_curve_;
  FixedDigitalLevelEstimator level_estimator_;
  ApmDataDumper* const apm_data_dumper_ = nullptr;

  // Work array containing the sub-frame scaling factors to be interpolated.
  float scaling_factors_[kSubFramesInFrame + 1];
  RTC_VIEW(float) scaling_factors_view_;
  float per_sample_scaling_factors_[kMaximalNumberOfSamplesPerChannel];
  RTC_VIEW(float) per_sample_scaling_factors_view_;
  float last_scaling_factor_ = 1.f;

  RTC_DISALLOW_COPY_AND_ASSIGN(GainCurveApplier);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_GAIN_CURVE_APPLIER_H_
