/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/aec3/subband_erle_estimator.h"

#include <algorithm>
// #include <functional>

#include "rtc_base/checks.h"
#include "rtc_base/numerics/safe_minmax.h"
// #include "system_wrappers/include/field_trial.h"

namespace webrtc {

namespace {

constexpr float kX2BandEnergyThreshold = 44015068.0f;
constexpr int kBlocksToHoldErle = 100;
constexpr int kBlocksForOnsetDetection = kBlocksToHoldErle + 150;
constexpr int kPointsToAccumulate = 6;

std::array<float, kFftLengthBy2Plus1> SetMaxErleBands(float max_erle_l,
                                                      float max_erle_h) {
  std::array<float, kFftLengthBy2Plus1> max_erle;
  std::fill(max_erle.begin(), max_erle.begin() + kFftLengthBy2 / 2, max_erle_l);
  std::fill(max_erle.begin() + kFftLengthBy2 / 2, max_erle.end(), max_erle_h);
  return max_erle;
}

bool EnableMinErleDuringOnsets() {
  return !true/* field_trial::IsEnabled("WebRTC-Aec3MinErleDuringOnsetsKillSwitch") */;
}

}  // namespace

SubbandErleEstimator::SubbandErleEstimator(const EchoCanceller3Config& config,
                                           size_t num_capture_channels)
    : use_onset_detection_(config.erle.onset_detection),
      min_erle_(config.erle.min),
      max_erle_(SetMaxErleBands(config.erle.max_l, config.erle.max_h)),
      use_min_erle_during_onsets_(EnableMinErleDuringOnsets()),
      accum_spectra_(num_capture_channels),
      erle_(num_capture_channels),
      erle_onsets_(num_capture_channels),
      coming_onset_(num_capture_channels),
      hold_counters_(num_capture_channels) {
  Reset();
}

SubbandErleEstimator::~SubbandErleEstimator() {}

void SubbandErleEstimator::Reset() {
  for (auto& erle : erle_) {
    erle.fill(min_erle_);
  }
  for (size_t ch = 0; ch < erle_onsets_.size(); ++ch) {
    erle_onsets_[ch].fill(min_erle_);
    coming_onset_[ch].fill(true);
    hold_counters_[ch].fill(0);
  }
  ResetAccumulatedSpectra();
}

void SubbandErleEstimator::Update(
    RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ X2,
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& Y2,
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& E2,
    const std::vector<bool>& converged_filters) {
  UpdateAccumulatedSpectra(X2, Y2, E2, converged_filters);
  UpdateBands(converged_filters);

  if (use_onset_detection_) {
    DecreaseErlePerBandForLowRenderSignals();
  }

  for (auto& erle : erle_) {
    erle[0] = erle[1];
    erle[kFftLengthBy2] = erle[kFftLengthBy2 - 1];
  }
}

void SubbandErleEstimator::Dump(
    const std::unique_ptr<ApmDataDumper>& data_dumper) const {
  data_dumper->DumpRaw(
      "aec3_erle_onset", RTC_MAKE_VIEW(const float)(ErleOnsets()[0]));
}

void SubbandErleEstimator::UpdateBands(
    const std::vector<bool>& converged_filters) {
  const int num_capture_channels = static_cast<int>(accum_spectra_.Y2.size());
  for (int ch = 0; ch < num_capture_channels; ++ch) {
    // Note that the use of the converged_filter flag already imposed
    // a minimum of the erle that can be estimated as that flag would
    // be false if the filter is performing poorly.
    if (!converged_filters[ch]) {
      continue;
    }

    std::array<float, kFftLengthBy2> new_erle;
    std::array<bool, kFftLengthBy2> is_erle_updated;
    is_erle_updated.fill(false);

    for (size_t k = 1; k < kFftLengthBy2; ++k) {
      if (accum_spectra_.num_points[ch] == kPointsToAccumulate &&
          accum_spectra_.E2[ch][k] > 0.f) {
        new_erle[k] = accum_spectra_.Y2[ch][k] / accum_spectra_.E2[ch][k];
        is_erle_updated[k] = true;
      }
    }

    if (use_onset_detection_) {
      for (size_t k = 1; k < kFftLengthBy2; ++k) {
        if (is_erle_updated[k] && !accum_spectra_.low_render_energy[ch][k]) {
          if (coming_onset_[ch][k]) {
            coming_onset_[ch][k] = false;
            if (!use_min_erle_during_onsets_) {
              float alpha = new_erle[k] < erle_onsets_[ch][k] ? 0.3f : 0.15f;
              erle_onsets_[ch][k] = rtc::SafeClamp<float>(
                  erle_onsets_[ch][k] +
                      alpha * (new_erle[k] - erle_onsets_[ch][k]),
                  min_erle_, max_erle_[k]);
            }
          }
          hold_counters_[ch][k] = kBlocksForOnsetDetection;
        }
      }
    }

    for (size_t k = 1; k < kFftLengthBy2; ++k) {
      if (is_erle_updated[k]) {
        float alpha = 0.05f;
        if (new_erle[k] < erle_[ch][k]) {
          alpha = accum_spectra_.low_render_energy[ch][k] ? 0.f : 0.1f;
        }
        erle_[ch][k] =
            rtc::SafeClamp<float>(erle_[ch][k] + alpha * (new_erle[k] - erle_[ch][k]),
                           min_erle_, max_erle_[k]);
      }
    }
  }
}

void SubbandErleEstimator::DecreaseErlePerBandForLowRenderSignals() {
  const int num_capture_channels = static_cast<int>(accum_spectra_.Y2.size());
  for (int ch = 0; ch < num_capture_channels; ++ch) {
    for (size_t k = 1; k < kFftLengthBy2; ++k) {
      --hold_counters_[ch][k];
      if (hold_counters_[ch][k] <=
          (kBlocksForOnsetDetection - kBlocksToHoldErle)) {
        if (erle_[ch][k] > erle_onsets_[ch][k]) {
          erle_[ch][k] = std::max(erle_onsets_[ch][k], 0.97f * erle_[ch][k]);
          RTC_DCHECK_LE(min_erle_, erle_[ch][k]);
        }
        if (hold_counters_[ch][k] <= 0) {
          coming_onset_[ch][k] = true;
          hold_counters_[ch][k] = 0;
        }
      }
    }
  }
}

void SubbandErleEstimator::ResetAccumulatedSpectra() {
  for (size_t ch = 0; ch < erle_onsets_.size(); ++ch) {
    accum_spectra_.Y2[ch].fill(0.f);
    accum_spectra_.E2[ch].fill(0.f);
    accum_spectra_.num_points[ch] = 0;
    accum_spectra_.low_render_energy[ch].fill(false);
  }
}

void SubbandErleEstimator::UpdateAccumulatedSpectra(
    RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ X2,
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& Y2,
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& E2,
    const std::vector<bool>& converged_filters) {
  auto& st = accum_spectra_;
  RTC_DCHECK_EQ(st.E2.size(), E2.size());
  RTC_DCHECK_EQ(st.E2.size(), E2.size());
  const int num_capture_channels = static_cast<int>(Y2.size());
  for (int ch = 0; ch < num_capture_channels; ++ch) {
    // Note that the use of the converged_filter flag already imposed
    // a minimum of the erle that can be estimated as that flag would
    // be false if the filter is performing poorly.
    if (!converged_filters[ch]) {
      continue;
    }

    if (st.num_points[ch] == kPointsToAccumulate) {
      st.num_points[ch] = 0;
      st.Y2[ch].fill(0.f);
      st.E2[ch].fill(0.f);
      st.low_render_energy[ch].fill(false);
    }

    std::transform(Y2[ch].begin(), Y2[ch].end(), st.Y2[ch].begin(),
                   st.Y2[ch].begin(), std::plus<float>());
    std::transform(E2[ch].begin(), E2[ch].end(), st.E2[ch].begin(),
                   st.E2[ch].begin(), std::plus<float>());

    for (size_t k = 0; k < X2.size(); ++k) {
      st.low_render_energy[ch][k] =
          st.low_render_energy[ch][k] || X2[k] < kX2BandEnergyThreshold;
    }

    ++st.num_points[ch];
  }
}

}  // namespace webrtc
