/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/aec3/stationarity_estimator.h"

#include <algorithm>
// #include <array>

#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/spectrum_buffer.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/atomic_ops.h"

namespace webrtc {

namespace {
constexpr float kMinNoisePower = 10.f;
constexpr int kHangoverBlocks = kNumBlocksPerSecond / 20;
constexpr int kNBlocksAverageInitPhase = 20;
constexpr int kNBlocksInitialPhase = kNumBlocksPerSecond * 2.;
}  // namespace

StationarityEstimator::StationarityEstimator()
    : data_dumper_(
          new ApmDataDumper(rtc::AtomicOps::Increment(&instance_count_))) {
  Reset();
}

StationarityEstimator::~StationarityEstimator() {}

void StationarityEstimator::Reset() {
  noise_.Reset();
  hangovers_view_.fill(0);
  stationarity_flags_view_.fill(false);
}

// Update just the noise estimator. Usefull until the delay is known
void StationarityEstimator::UpdateNoiseEstimator(
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& spectrum) {
  noise_.Update(spectrum);
  data_dumper_->DumpRaw(
      "aec3_stationarity_noise_spectrum", noise_.Spectrum());
  data_dumper_->DumpRaw(
      "aec3_stationarity_is_block_stationary", IsBlockStationary());
}

void StationarityEstimator::UpdateStationarityFlags(
    const SpectrumBuffer& spectrum_buffer,
    RTC_VIEW(const float) render_reverb_contribution_spectrum,
    int idx_current,
    int num_lookahead) {
  int indexes[kWindowLength];
  RTC_VIEW(int) indexes_view = RTC_MAKE_VIEW(int)(indexes);
  int num_lookahead_bounded = std::min(num_lookahead, kWindowLength - 1);
  int idx = idx_current;

  if (num_lookahead_bounded < kWindowLength - 1) {
    int num_lookback = (kWindowLength - 1) - num_lookahead_bounded;
    idx = spectrum_buffer.OffsetIndex(idx_current, num_lookback);
  }
  // For estimating the stationarity properties of the current frame, the
  // power for each band is accumulated for several consecutive spectra in the
  // method EstimateBandStationarity.
  // In order to avoid getting the indexes_view of the spectra for every band with
  // its associated overhead, those indexes_view are stored in an array and then use
  // when the estimation is done.
  indexes_view[0] = idx;
  for (size_t k = 1; k < indexes_view.size(); ++k) {
    indexes_view[k] = spectrum_buffer.DecIndex(indexes_view[k - 1]);
  }
  RTC_DCHECK_EQ(
      spectrum_buffer.DecIndex(indexes_view[kWindowLength - 1]),
      spectrum_buffer.OffsetIndex(idx_current, -(num_lookahead_bounded + 1)));

  for (size_t k = 0; k < stationarity_flags_view_.size(); ++k) {
    stationarity_flags_view_[k] = EstimateBandStationarity(
        spectrum_buffer, render_reverb_contribution_spectrum, indexes_view, k);
  }
  UpdateHangover();
  SmoothStationaryPerFreq();
}

bool StationarityEstimator::IsBlockStationary() const {
  float acum_stationarity = 0.f;
  RTC_DCHECK_EQ(stationarity_flags_view_.size(), kFftLengthBy2Plus1);
  for (size_t band = 0; band < stationarity_flags_view_.size(); ++band) {
    bool st = IsBandStationary(band);
    acum_stationarity += static_cast<float>(st);
  }
  return ((acum_stationarity * (1.f / kFftLengthBy2Plus1)) > 0.75f);
}

bool StationarityEstimator::EstimateBandStationarity(
    const SpectrumBuffer& spectrum_buffer,
    RTC_VIEW(const float) average_reverb,
    RTC_VIEW(const int) /* kWindowLength */ indexes_view,
    size_t band) const {
  constexpr float kThrStationarity = 10.f;
  float acum_power = 0.f;
  const int num_render_channels =
      static_cast<int>(spectrum_buffer.buffer[0].size());
  const float one_by_num_channels = 1.f / num_render_channels;
  for (auto idx : indexes_view) {
    for (int ch = 0; ch < num_render_channels; ++ch) {
      acum_power += spectrum_buffer.buffer[idx][ch][band] * one_by_num_channels;
    }
  }
  acum_power += average_reverb[band];
  float noise = kWindowLength * GetStationarityPowerBand(band);
  RTC_CHECK_LT(0.f, noise);
  bool stationary = acum_power < kThrStationarity * noise;
  data_dumper_->DumpRaw("aec3_stationarity_long_ratio", acum_power / noise);
  return stationary;
}

bool StationarityEstimator::AreAllBandsStationary() {
  for (auto b : stationarity_flags_view_) {
    if (!b)
      return false;
  }
  return true;
}

void StationarityEstimator::UpdateHangover() {
  bool reduce_hangover = AreAllBandsStationary();
  for (size_t k = 0; k < stationarity_flags_view_.size(); ++k) {
    if (!stationarity_flags_view_[k]) {
      hangovers_view_[k] = kHangoverBlocks;
    } else if (reduce_hangover) {
      hangovers_view_[k] = std::max(hangovers_view_[k] - 1, 0);
    }
  }
}

void StationarityEstimator::SmoothStationaryPerFreq() {
  bool all_ahead_stationary_smooth[kFftLengthBy2Plus1];
  for (size_t k = 1; k < kFftLengthBy2Plus1 - 1; ++k) {
    all_ahead_stationary_smooth[k] = stationarity_flags_view_[k - 1] &&
                                     stationarity_flags_view_[k] &&
                                     stationarity_flags_view_[k + 1];
  }

  all_ahead_stationary_smooth[0] = all_ahead_stationary_smooth[1];
  all_ahead_stationary_smooth[kFftLengthBy2Plus1 - 1] =
      all_ahead_stationary_smooth[kFftLengthBy2Plus1 - 2];

  stationarity_flags_view_ = all_ahead_stationary_smooth;
}

int StationarityEstimator::instance_count_ = 0;

StationarityEstimator::NoiseSpectrum::NoiseSpectrum() {
  Reset();
}

StationarityEstimator::NoiseSpectrum::~NoiseSpectrum() {}

void StationarityEstimator::NoiseSpectrum::Reset() {
  block_counter_ = 0;
  noise_spectrum_view_.fill(kMinNoisePower);
}

void StationarityEstimator::NoiseSpectrum::Update(
    const std::vector<std::array<float, kFftLengthBy2Plus1>>& spectrum) {
  RTC_DCHECK_LE(1, spectrum[0].size());
  const int num_render_channels = static_cast<int>(spectrum.size());

  float avg_spectrum_data[kFftLengthBy2Plus1];
  RTC_VIEW(float) avg_spectrum_data_view = RTC_MAKE_VIEW(float)(avg_spectrum_data);
  RTC_VIEW(const float) avg_spectrum;
  if (num_render_channels == 1) {
    avg_spectrum = spectrum[0];
  } else {
    // For multiple channels, average the channel spectra before passing to the
    // noise spectrum estimator.
    avg_spectrum = RTC_MAKE_VIEW(const float)(avg_spectrum_data);
    std::copy(spectrum[0].begin(), spectrum[0].end(),
              avg_spectrum_data_view.begin());
    for (int ch = 1; ch < num_render_channels; ++ch) {
      for (size_t k = 1; k < kFftLengthBy2Plus1; ++k) {
        avg_spectrum_data_view[k] += spectrum[ch][k];
      }
    }

    const float one_by_num_channels = 1.f / num_render_channels;
    for (size_t k = 1; k < kFftLengthBy2Plus1; ++k) {
      avg_spectrum_data[k] *= one_by_num_channels;
    }
  }

  ++block_counter_;
  float alpha = GetAlpha();
  for (size_t k = 0; k < kFftLengthBy2Plus1; ++k) {
    if (block_counter_ <= kNBlocksAverageInitPhase) {
      noise_spectrum_view_[k] += (1.f / kNBlocksAverageInitPhase) * avg_spectrum[k];
    } else {
      noise_spectrum_view_[k] =
          UpdateBandBySmoothing(avg_spectrum[k], noise_spectrum_view_[k], alpha);
    }
  }
}

float StationarityEstimator::NoiseSpectrum::GetAlpha() const {
  constexpr float kAlpha = 0.004f;
  constexpr float kAlphaInit = 0.04f;
  constexpr float kTiltAlpha = (kAlphaInit - kAlpha) / kNBlocksInitialPhase;

  if (block_counter_ > (kNBlocksInitialPhase + kNBlocksAverageInitPhase)) {
    return kAlpha;
  } else {
    return kAlphaInit -
           kTiltAlpha * (block_counter_ - kNBlocksAverageInitPhase);
  }
}

float StationarityEstimator::NoiseSpectrum::UpdateBandBySmoothing(
    float power_band,
    float power_band_noise,
    float alpha) const {
  float power_band_noise_updated = power_band_noise;
  if (power_band_noise < power_band) {
    RTC_DCHECK_GT(power_band, 0.f);
    float alpha_inc = alpha * (power_band_noise / power_band);
    if (block_counter_ > kNBlocksInitialPhase) {
      if (10.f * power_band_noise < power_band) {
        alpha_inc *= 0.1f;
      }
    }
    power_band_noise_updated += alpha_inc * (power_band - power_band_noise);
  } else {
    power_band_noise_updated += alpha * (power_band - power_band_noise);
    power_band_noise_updated =
        std::max(power_band_noise_updated, kMinNoisePower);
  }
  return power_band_noise_updated;
}

}  // namespace webrtc
