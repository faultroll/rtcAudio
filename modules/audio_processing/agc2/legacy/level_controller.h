/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_LEVEL_CONTROLLER_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_LEVEL_CONTROLLER_H_

#include <memory>
#include <vector>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/optional.h"
#include "modules/audio_processing/agc2/agc2_config.h"
#include "modules/audio_processing/agc2/legacy/gain_applier.h"
#include "modules/audio_processing/agc2/legacy/gain_selector.h"
#include "modules/audio_processing/agc2/noise_level_estimator.h"
#include "modules/audio_processing/agc2/legacy/peak_level_estimator.h"
#include "modules/audio_processing/agc2/legacy/saturating_gain_estimator.h"
#include "modules/audio_processing/agc2/signal_classifier.h"
#include "modules/audio_processing/include/common.h"

namespace webrtc {

class ApmDataDumper;
class AudioBuffer;

class LevelController {
 public:
  LevelController();
  ~LevelController();

  void Initialize(int sample_rate_hz);
  void Process(AudioBuffer* audio);
  float GetLastGain() { return last_gain_; }

  // TODO(peah): This method is a temporary solution as the the aim is to
  // instead apply the config inside the constructor. Therefore this is likely
  // to change.
  void ApplyConfig(const Agc2Config& config);

 private:
  class Metrics {
   public:
    Metrics() { Initialize(AudioProcessing::kSampleRate48kHz); }
    void Initialize(int sample_rate_hz);
    void Update(float long_term_peak_level,
                float noise_level,
                float gain,
                float frame_peak_level);

   private:
    void Reset();

    size_t metrics_frame_counter_;
    float gain_sum_;
    float peak_level_sum_;
    float noise_energy_sum_;
    float max_gain_;
    float max_peak_level_;
    float max_noise_energy_;
    float frame_length_;
  };

  std::unique_ptr<ApmDataDumper> data_dumper_;
  GainSelector gain_selector_;
  GainApplier gain_applier_;
  SignalClassifier signal_classifier_;
  NoiseLevelEstimator noise_level_estimator_;
  PeakLevelEstimator peak_level_estimator_;
  SaturatingGainEstimator saturating_gain_estimator_;
  Metrics metrics_;
  rtc::Optional<int> sample_rate_hz_;
  static int instance_count_;
  float dc_level_[2];
  float dc_forgetting_factor_;
  float last_gain_;
  bool gain_jumpstart_ = false;
  Agc2Config config_;

  RTC_DISALLOW_COPY_AND_ASSIGN(LevelController);
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_LEVEL_CONTROLLER_H_
