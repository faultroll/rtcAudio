/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_

#include "modules/audio_processing/agc2/adaptive_digital_gain_applier.h"
#include "modules/audio_processing/agc2/adaptive_mode_level_estimator.h"
#include "modules/audio_processing/agc2/noise_level_estimator.h"
#include "modules/audio_processing/agc2/vad_with_level.h"
#include "modules/audio_processing/include/audio_frame_view.h"
#include "modules/audio_processing/agc2/agc2_config.h"

namespace webrtc {
class ApmDataDumper;

// Adaptive digital gain controller.
// TODO(crbug.com/webrtc/7494): Unify with `AdaptiveDigitalGainApplier`.
class AdaptiveAgc {
 public:
  explicit AdaptiveAgc(ApmDataDumper* apm_data_dumper);
  // TODO(crbug.com/webrtc/7494): Remove ctor above.
  AdaptiveAgc(ApmDataDumper* apm_data_dumper,
              const Agc2Config& config);
  ~AdaptiveAgc();

  // Analyzes `frame` and applies a digital adaptive gain to it. Takes into
  // account the envelope measured by the limiter.
  // TODO(crbug.com/webrtc/7494): Make the class depend on the limiter.
  void Process(AudioFrameView<float> frame, float limiter_envelope);
  void Reset();

 private:
  AdaptiveModeLevelEstimator speech_level_estimator_;
  VadLevelAnalyzer vad_;
  AdaptiveDigitalGainApplier gain_applier_;
  ApmDataDumper* const apm_data_dumper_;
  NoiseLevelEstimator noise_level_estimator_;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_AGC_H_
