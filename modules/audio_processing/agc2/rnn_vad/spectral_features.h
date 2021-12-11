/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_H_

// #include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/agc2/rnn_vad/common.h"
#include "modules/audio_processing/agc2/rnn_vad/ring_buffer.h"
#include "modules/audio_processing/agc2/rnn_vad/spectral_features_internal.h"
#include "modules/audio_processing/agc2/rnn_vad/symmetric_matrix_buffer.h"
#include "modules/audio_processing/utility/pffft_wrapper.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rnn_vad {

// Class to compute spectral features.
class SpectralFeaturesExtractor {
 public:
  SpectralFeaturesExtractor();
  ~SpectralFeaturesExtractor();
  // Resets the internal state of the feature extractor.
  void Reset();
  // Analyzes a pair of reference and lagged frames from the pitch buffer,
  // detects silence and computes features. If silence is detected, the output
  // is neither computed nor written.
  bool CheckSilenceComputeFeatures(
      RTC_VIEW(const float) /* kFrameSize20ms24kHz */ reference_frame,
      RTC_VIEW(const float) /* kFrameSize20ms24kHz */ lagged_frame,
      RTC_VIEW(float) /* kNumBands - kNumLowerBands */ higher_bands_cepstrum,
      RTC_VIEW(float) /* kNumLowerBands */ average,
      RTC_VIEW(float) /* kNumLowerBands */ first_derivative,
      RTC_VIEW(float) /* kNumLowerBands */ second_derivative,
      RTC_VIEW(float) /* kNumLowerBands */ bands_cross_corr,
      float* variability);

 private:
  void ComputeAvgAndDerivatives(
      RTC_VIEW(float) /* kNumLowerBands */ average,
      RTC_VIEW(float) /* kNumLowerBands */ first_derivative,
      RTC_VIEW(float) /* kNumLowerBands */ second_derivative) const;
  void ComputeNormalizedCepstralCorrelation(
      RTC_VIEW(float) /* kNumLowerBands */ bands_cross_corr);
  float ComputeVariability() const;

  const float half_window_[kFrameSize20ms24kHz / 2];
  RTC_VIEW(const float) half_window_view_;
  Pffft fft_;
  std::unique_ptr<Pffft::FloatBuffer> fft_buffer_;
  std::unique_ptr<Pffft::FloatBuffer> reference_frame_fft_;
  std::unique_ptr<Pffft::FloatBuffer> lagged_frame_fft_;
  SpectralCorrelator spectral_correlator_;
  float reference_frame_bands_energy_[kOpusBands24kHz];
  float lagged_frame_bands_energy_[kOpusBands24kHz];
  float bands_cross_corr_[kOpusBands24kHz];
  const float dct_table_[kNumBands * kNumBands];
  RTC_VIEW(const float) dct_table_view_;
  RingBuffer<float, kNumBands, kCepstralCoeffsHistorySize>
      cepstral_coeffs_ring_buf_;
  SymmetricMatrixBuffer<float, kCepstralCoeffsHistorySize> cepstral_diffs_buf_;

  RTC_DISALLOW_COPY_AND_ASSIGN(SpectralFeaturesExtractor);
};

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_H_
