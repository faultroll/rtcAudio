/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_NOISE_SUPPRESSOR_H_
#define MODULES_AUDIO_PROCESSING_NS_NOISE_SUPPRESSOR_H_

#include <memory>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/ns2/noise_estimator.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/ns_config.h"
#include "modules/audio_processing/ns2/ns_fft.h"
#include "modules/audio_processing/ns2/speech_probability_estimator.h"
#include "modules/audio_processing/ns2/wiener_filter.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Class for suppressing noise in a signal.
class NoiseSuppressor {
 public:
  NoiseSuppressor(const NsConfig& config,
                  size_t sample_rate_hz,
                  size_t num_channels);

  // Analyses the signal (typically applied before the AEC to avoid analyzing
  // any comfort noise signal).
  void Analyze(const AudioBuffer& audio);

  // Applies noise suppression.
  void Process(AudioBuffer* audio);

 private:
  const size_t num_bands_;
  const size_t num_channels_;
  const SuppressionParams suppression_params_;
  int32_t num_analyzed_frames_ = -1;
  NrFft fft_;

  struct ChannelState {
    ChannelState(const SuppressionParams& suppression_params, size_t num_bands);

    SpeechProbabilityEstimator speech_probability_estimator;
    WienerFilter wiener_filter;
    NoiseEstimator noise_estimator;
    float prev_analysis_signal_spectrum[kFftSizeBy2Plus1];
    float analyze_analysis_memory[kFftSize - kNsFrameSize];
    float process_analysis_memory[kOverlapSize];
    float process_synthesis_memory[kOverlapSize];
    std::vector<std::array<float, kOverlapSize>> process_delay_memory;
  };

  struct FilterBankState {
    float real[kFftSize];
    float imag[kFftSize];
    float extended_frame[kFftSize];
  };

  std::vector<FilterBankState> filter_bank_states_heap_;
  std::vector<float> upper_band_gains_heap_;
  std::vector<float> energies_before_filtering_heap_;
  std::vector<float> gain_adjustments_heap_;
  std::vector<std::unique_ptr<ChannelState>> channels_;

  // Aggregates the Wiener filters into a single filter to use.
  void AggregateWienerFilters(
      RTC_VIEW(float) /* kFftSizeBy2Plus1 */ filter) const;
  
  RTC_DISALLOW_COPY_AND_ASSIGN(NoiseSuppressor);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_NOISE_SUPPRESSOR_H_
