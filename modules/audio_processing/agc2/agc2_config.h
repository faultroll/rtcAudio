/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_AGC2_CONFIG_H_
#define MODULES_AUDIO_PROCESSING_AGC2_AGC2_CONFIG_H_

#include <string>
#include <sstream>

namespace webrtc {

// Enables the next generation AGC functionality. This feature replaces the
// standard methods of gain control in the previous AGC. Enabling this
// submodule enables an adaptive digital AGC followed by a limiter. By
// setting |fixed_gain_db|, the limiter can be turned into a compressor that
// first applies a fixed gain. The adaptive digital AGC can be turned off by
// setting |adaptive_digital_mode=false|.
struct Agc2Config {
  // Validates a config.
  static bool Validate(const Agc2Config& config) {
    return (config.initial_peak_level_dbfs <
                std::numeric_limits<float>::epsilon() &&
            config.initial_peak_level_dbfs >
                -(100.f + std::numeric_limits<float>::epsilon()) &&
            config.fixed_gain_db >= 0.f &&
            config.fixed_digital.gain_db >= 0.f &&
            config.fixed_digital.gain_db < 50.f &&
            config.adaptive_digital.extra_saturation_margin_db >= 0.f &&
            config.adaptive_digital.extra_saturation_margin_db <= 100.f);
  }
  // Dumps a config to a string.
  static std::string ToString(const Agc2Config& config) {
    std::stringstream ss;
    std::string adaptive_digital_level_estimator;
    switch (config.adaptive_digital.level_estimator) {
      case kRms:
        adaptive_digital_level_estimator = "RMS";
        break;
      case kPeak:
        adaptive_digital_level_estimator = "peak";
        break;
    }
    // clang-format off
    // clang formatting doesn't respect custom nested style.
    ss << "{"
       << "enabled: " << (config.enabled ? "true" : "false") << ", "
       << "initial_peak_level_dbfs: " << config.initial_peak_level_dbfs << ", "
       << "fixed_gain_dB: " << config.fixed_gain_db
       << "}";
    ss << "{"
          "enabled: " << (config.enabled ? "true" : "false") << ", "
          "fixed_digital: {gain_db: " << config.fixed_digital.gain_db << "}, "
          "adaptive_digital: {"
            "enabled: "
              << (config.adaptive_digital.enabled ? "true" : "false") << ", "
            "level_estimator: {"
              "type: " << adaptive_digital_level_estimator << ", "
              "adjacent_speech_frames_threshold: "
                << config.adaptive_digital
                    .level_estimator_adjacent_speech_frames_threshold << ", "
              "initial_saturation_margin_db: "
                << config.adaptive_digital.initial_saturation_margin_db << ", "
              "extra_saturation_margin_db: "
                << config.adaptive_digital.extra_saturation_margin_db << "}, "
            "gain_applier: {"
              "adjacent_speech_frames_threshold: "
                << config.adaptive_digital
                    .gain_applier_adjacent_speech_frames_threshold << ", "
              "max_gain_change_db_per_second: "
                << config.adaptive_digital.max_gain_change_db_per_second << ", "
              "max_output_noise_level_dbfs: "
                << config.adaptive_digital.max_output_noise_level_dbfs << "}"
          "}"
          "}";
    // clang-format on
    return ss.str();
  }

  bool enabled = false;
  // old
  bool adaptive_digital_mode = true;
  float initial_peak_level_dbfs = -10.f;
  float fixed_gain_db = 0.f;
  // new
  enum LevelEstimator { kRms, kPeak };
  struct {
    float gain_db = 0.f;
  } fixed_digital;
  struct {
    bool enabled = false;
    float vad_probability_attack = 1.f;
    LevelEstimator level_estimator = kRms;
    int level_estimator_adjacent_speech_frames_threshold = 1;
    // TODO(crbug.com/webrtc/7494): Remove `use_saturation_protector`.
    bool use_saturation_protector = true;
    float initial_saturation_margin_db = 20.f;
    float extra_saturation_margin_db = 2.f;
    int gain_applier_adjacent_speech_frames_threshold = 1;
    float max_gain_change_db_per_second = 3.f;
    float max_output_noise_level_dbfs = -50.f;
  } adaptive_digital;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_AGC2_CONFIG_H_
