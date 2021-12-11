/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/aec3/coarse_filter_update_gain.h"

#include <algorithm>
// #include <functional>

#include "rtc_base/checks.h"

namespace webrtc {

CoarseFilterUpdateGain::CoarseFilterUpdateGain(
    const EchoCanceller3Config::Filter::CoarseConfiguration& config,
    size_t config_change_duration_blocks)
    : config_change_duration_blocks_(
          static_cast<int>(config_change_duration_blocks)) {
  SetConfig(config, true);
  RTC_DCHECK_LT(0, config_change_duration_blocks_);
  one_by_config_change_duration_blocks_ = 1.f / config_change_duration_blocks_;
}

void CoarseFilterUpdateGain::HandleEchoPathChange() {
  poor_signal_excitation_counter_ = 0;
  call_counter_ = 0;
}

void CoarseFilterUpdateGain::Compute(
    RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ render_power,
    const RenderSignalAnalyzer& render_signal_analyzer,
    const FftData& E_coarse,
    size_t size_partitions,
    bool saturated_capture_signal,
    FftData* G) {
  RTC_DCHECK(G);
  ++call_counter_;

  UpdateCurrentConfig();

  if (render_signal_analyzer.PoorSignalExcitation()) {
    poor_signal_excitation_counter_ = 0;
  }

  // Do not update the filter if the render is not sufficiently excited.
  if (++poor_signal_excitation_counter_ < size_partitions ||
      saturated_capture_signal || call_counter_ <= size_partitions) {
    G->re_view.fill(0.f);
    G->im_view.fill(0.f);
    return;
  }

  // Compute mu.
  float mu[kFftLengthBy2Plus1];
  RTC_VIEW(float) mu_view = RTC_MAKE_VIEW(float)(mu);
  const auto& X2 = render_power;
  for (size_t k = 0; k < kFftLengthBy2Plus1; ++k) {
    if (X2[k] > current_config_.noise_gate) {
      mu_view[k] = current_config_.rate / X2[k];
    } else {
      mu_view[k] = 0.f;
    }
  }

  // Avoid updating the filter close to narrow bands in the render signals.
  render_signal_analyzer.MaskRegionsAroundNarrowBands(mu_view);

  // G = mu * E * X2.
  for (size_t k = 0; k < kFftLengthBy2Plus1; ++k) {
    G->re_view[k] = mu_view[k] * E_coarse.re_view[k];
    G->im_view[k] = mu_view[k] * E_coarse.im_view[k];
  }
}

void CoarseFilterUpdateGain::UpdateCurrentConfig() {
  RTC_DCHECK_GE(config_change_duration_blocks_, config_change_counter_);
  if (config_change_counter_ > 0) {
    if (--config_change_counter_ > 0) {
      auto average = [](float from, float to, float from_weight) {
        return from * from_weight + to * (1.f - from_weight);
      };

      float change_factor =
          config_change_counter_ * one_by_config_change_duration_blocks_;

      current_config_.rate =
          average(old_target_config_.rate, target_config_.rate, change_factor);
      current_config_.noise_gate =
          average(old_target_config_.noise_gate, target_config_.noise_gate,
                  change_factor);
    } else {
      current_config_ = old_target_config_ = target_config_;
    }
  }
  RTC_DCHECK_LE(0, config_change_counter_);
}

}  // namespace webrtc
