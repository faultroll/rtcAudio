/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_RESIDUAL_ECHO_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AEC3_RESIDUAL_ECHO_ESTIMATOR_H_

// #include <array>
#include <memory>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/echo_canceller3_config.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/aec_state.h"
#include "modules/audio_processing/aec3/render_buffer.h"
#include "modules/audio_processing/aec3/reverb_model.h"
#include "modules/audio_processing/aec3/spectrum_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

class ResidualEchoEstimator {
 public:
  ResidualEchoEstimator(const EchoCanceller3Config& config,
                        size_t num_render_channels);
  ~ResidualEchoEstimator();


  void Estimate(
      const AecState& aec_state,
      const RenderBuffer& render_buffer,
      const std::vector<std::array<float, kFftLengthBy2Plus1>>& S2_linear,
      const std::vector<std::array<float, kFftLengthBy2Plus1>>& Y2,
      std::vector<std::array<float, kFftLengthBy2Plus1>>& R2);

 private:
  enum class ReverbType { kLinear, kNonLinear };

  // Resets the state.
  void Reset();

  // Updates estimate for the power of the stationary noise component in the
  // render signal.
  void UpdateRenderNoisePower(const RenderBuffer& render_buffer);

  // Adds the estimated unmodelled echo power to the residual echo power
  // estimate.
  void AddReverb(ReverbType reverb_type,
                 const AecState& aec_state,
                 const RenderBuffer& render_buffer,
                 std::vector<std::array<float, kFftLengthBy2Plus1>>& R2);

  // Gets the echo path gain to apply.
  float GetEchoPathGain(const AecState& aec_state,
                        bool gain_for_early_reflections) const;

  const EchoCanceller3Config config_;
  const size_t num_render_channels_;
  const float early_reflections_transparent_mode_gain_;
  const float late_reflections_transparent_mode_gain_;
  const float early_reflections_general_gain_;
  const float late_reflections_general_gain_;
  float X2_noise_floor_[kFftLengthBy2Plus1];
  RTC_VIEW(float) X2_noise_floor_view_ = RTC_MAKE_VIEW(float)(X2_noise_floor_);
  int X2_noise_floor_counter_[kFftLengthBy2Plus1];
  RTC_VIEW(int) X2_noise_floor_counter_view_ = RTC_MAKE_VIEW(int)(X2_noise_floor_counter_);
  ReverbModel echo_reverb_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(ResidualEchoEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_RESIDUAL_ECHO_ESTIMATOR_H_
