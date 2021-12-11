/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_INTERNAL_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_INTERNAL_H_

#include <stddef.h>

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/agc2/rnn_vad/common.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rnn_vad {

// At a sample rate of 24 kHz, the last 3 Opus bands are beyond the Nyquist
// frequency. However, band #19 gets the contributions from band #18 because
// of the symmetric triangular filter with peak response at 12 kHz.
constexpr size_t kOpusBands24kHz = 20;
static_assert(kOpusBands24kHz < kNumBands,
              "The number of bands at 24 kHz must be less than those defined "
              "in the Opus scale at 48 kHz.");

// Number of FFT frequency bins covered by each band in the Opus scale at a
// sample rate of 24 kHz for 20 ms frames.
// Declared here for unit testing.
constexpr std::array<int, kOpusBands24kHz - 1> GetOpusScaleNumBins24kHz20ms() {
  return {{4, 4, 4, 4, 4, 4, 4, 4, 8, 8, 8, 8, 16, 16, 16, 24, 24, 32, 48}};
}

// TODO(bugs.webrtc.org/10480): Move to a separate file.
// Class to compute band-wise spectral features in the Opus perceptual scale
// for 20 ms frames sampled at 24 kHz. The analysis methods apply triangular
// filters with peak response at the each band boundary.
class SpectralCorrelator {
 public:
  // Ctor.
  SpectralCorrelator();
  ~SpectralCorrelator();

  // Computes the band-wise spectral auto-correlations.
  // |x| must:
  //  - have size equal to |kFrameSize20ms24kHz|;
  //  - be encoded as vectors of interleaved real-complex FFT coefficients
  //    where x[1] = y[1] = 0 (the Nyquist frequency coefficient is omitted).
  void ComputeAutoCorrelation(
      RTC_VIEW(const float) x,
      RTC_VIEW(float) /* kOpusBands24kHz */ auto_corr) const;

  // Computes the band-wise spectral cross-correlations.
  // |x| and |y| must:
  //  - have size equal to |kFrameSize20ms24kHz|;
  //  - be encoded as vectors of interleaved real-complex FFT coefficients where
  //    x[1] = y[1] = 0 (the Nyquist frequency coefficient is omitted).
  void ComputeCrossCorrelation(
      RTC_VIEW(const float) x,
      RTC_VIEW(const float) y,
      RTC_VIEW(float) /* kOpusBands24kHz */ cross_corr) const;

 private:
  const std::vector<float> weights_;  // Weights for each Fourier coefficient.

  RTC_DISALLOW_COPY_AND_ASSIGN(SpectralCorrelator);
};

// TODO(bugs.webrtc.org/10480): Move to anonymous namespace in
// spectral_features.cc. Given a vector of Opus-bands energy coefficients,
// computes the log magnitude spectrum applying smoothing both over time and
// over frequency. Declared here for unit testing.
void ComputeSmoothedLogMagnitudeSpectrum(
    RTC_VIEW(const float) bands_energy,
    RTC_VIEW(float) /* kNumBands */ log_bands_energy);

// TODO(bugs.webrtc.org/10480): Move to anonymous namespace in
// spectral_features.cc. Creates a DCT table for arrays having size equal to
// |kNumBands|. Declared here for unit testing.
void ComputeDctTable(RTC_VIEW(float) /* kNumBands * kNumBands */ dct_table);

// TODO(bugs.webrtc.org/10480): Move to anonymous namespace in
// spectral_features.cc. Computes DCT for |in| given a pre-computed DCT table.
// In-place computation is not allowed and |out| can be smaller than |in| in
// order to only compute the first DCT coefficients. Declared here for unit
// testing.
void ComputeDct(RTC_VIEW(const float) in,
                RTC_VIEW(const float) /* kNumBands * kNumBands */ dct_table,
                RTC_VIEW(float) out);

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_SPECTRAL_FEATURES_INTERNAL_H_
