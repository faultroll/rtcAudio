/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_

#include <stddef.h>
// #include <type_traits>

#include "modules/audio_processing/agc2/agc2_common.h"
#include "modules/audio_processing/agc2/saturation_protector.h"
#include "modules/audio_processing/agc2/vad_with_level.h"
#include "modules/audio_processing/agc2/agc2_config.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
class ApmDataDumper;

// enum LevelEstimator { kRms, kPeak };

// Level estimator for the digital adaptive gain controller.
class AdaptiveModeLevelEstimator {
 public:
  explicit AdaptiveModeLevelEstimator(ApmDataDumper* apm_data_dumper);
  AdaptiveModeLevelEstimator(
      ApmDataDumper* apm_data_dumper,
      Agc2Config::LevelEstimator level_estimator,
      int adjacent_speech_frames_threshold,
      float initial_saturation_margin_db,
      float extra_saturation_margin_db);

  // Updates the level estimation.
  void Update(const VadLevelAnalyzer::Result& vad_data);
  // Returns the estimated speech plus noise level.
  float level_dbfs() const { return level_dbfs_; }
  // Returns true if the estimator is confident on its current estimate.
  bool IsConfident() const;

  void Reset();

 private:
  // Part of the level estimator state used for check-pointing and restore ops.
  struct LevelEstimatorState {
    bool operator==(const LevelEstimatorState& s) const;
    inline bool operator!=(const LevelEstimatorState& s) const {
      return !(*this == s);
    }
    struct Ratio {
      float numerator;
      float denominator;
      float GetRatio() const;
    };
    // TODO(crbug.com/webrtc/7494): Remove time_to_full_buffer_ms if redundant.
    int time_to_full_buffer_ms;
    Ratio level_dbfs;
    SaturationProtectorState saturation_protector;
  };
  /* static_assert(std::is_trivially_copyable<LevelEstimatorState>::value, ""); */

  void ResetLevelEstimatorState(LevelEstimatorState& state) const;

  void DumpDebugData() const;

  ApmDataDumper* const apm_data_dumper_;

  const Agc2Config::LevelEstimator level_estimator_type_;
  const int adjacent_speech_frames_threshold_;
  const float initial_saturation_margin_db_;
  const float extra_saturation_margin_db_;
  LevelEstimatorState preliminary_state_;
  LevelEstimatorState reliable_state_;
  float level_dbfs_;
  int num_adjacent_speech_frames_;

  RTC_DISALLOW_COPY_AND_ASSIGN(AdaptiveModeLevelEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_ADAPTIVE_MODE_LEVEL_ESTIMATOR_H_
