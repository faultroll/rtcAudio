/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_SUBTRACTOR_H_
#define MODULES_AUDIO_PROCESSING_AEC3_SUBTRACTOR_H_

#include <math.h>
#include <stddef.h>

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/echo_canceller3_config.h"
#include "modules/audio_processing/aec3/adaptive_fir_filter.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/aec3_fft.h"
#include "modules/audio_processing/aec3/aec_state.h"
#include "modules/audio_processing/aec3/coarse_filter_update_gain.h"
#include "modules/audio_processing/aec3/echo_path_variability.h"
#include "modules/audio_processing/aec3/refined_filter_update_gain.h"
#include "modules/audio_processing/aec3/render_buffer.h"
#include "modules/audio_processing/aec3/render_signal_analyzer.h"
#include "modules/audio_processing/aec3/subtractor_output.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
// #include "common_audio/third_party/ooura/fft_size_128/ooura_fft.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Proves linear echo cancellation functionality
class Subtractor {
 public:
  Subtractor(const EchoCanceller3Config& config,
             size_t num_render_channels,
             size_t num_capture_channels,
             ApmDataDumper* data_dumper,
             Aec3Optimization optimization);
  ~Subtractor();

  // Performs the echo subtraction.
  void Process(const RenderBuffer& render_buffer,
               const std::vector<std::vector<float>>& capture,
               const RenderSignalAnalyzer& render_signal_analyzer,
               const AecState& aec_state,
               RTC_VIEW(SubtractorOutput) outputs);

  void HandleEchoPathChange(const EchoPathVariability& echo_path_variability);

  // Exits the initial state.
  void ExitInitialState();

  // Returns the block-wise frequency responses for the refined adaptive
  // filters.
  const std::vector<std::vector<std::array<float, kFftLengthBy2Plus1>>>&
  FilterFrequencyResponses() const {
    return refined_frequency_responses_;
  }

  // Returns the estimates of the impulse responses for the refined adaptive
  // filters.
  const std::vector<std::vector<float>>& FilterImpulseResponses() const {
    return refined_impulse_responses_;
  }

  void DumpFilters() {
    data_dumper_->DumpRaw(
        "aec3_subtractor_h_refined",
        RTC_MAKE_VIEW(const float)(
            refined_impulse_responses_[0].data(),
            GetTimeDomainLength(
                refined_filters_[0]->max_filter_size_partitions())));

    refined_filters_[0]->DumpFilter("aec3_subtractor_H_refined");
    coarse_filter_[0]->DumpFilter("aec3_subtractor_H_coarse");
  }

 private:
  class FilterMisadjustmentEstimator {
   public:
    FilterMisadjustmentEstimator() {}
    ~FilterMisadjustmentEstimator() {}
    // Update the misadjustment estimator.
    void Update(const SubtractorOutput& output);
    // GetMisadjustment() Returns a recommended scale for the filter so the
    // prediction error energy gets closer to the energy that is seen at the
    // microphone input.
    float GetMisadjustment() const {
      RTC_DCHECK_GT(inv_misadjustment_, 0.0f);
      // It is not aiming to adjust all the estimated mismatch. Instead,
      // it adjusts half of that estimated mismatch.
      return 2.f / sqrtf(inv_misadjustment_);
    }
    // Returns true if the prediciton error energy is significantly larger
    // than the microphone signal energy and, therefore, an adjustment is
    // recommended.
    bool IsAdjustmentNeeded() const { return inv_misadjustment_ > 10.f; }
    void Reset();
    void Dump(ApmDataDumper* data_dumper) const;

   private:
    const int n_blocks_ = 4;
    int n_blocks_acum_ = 0;
    float e2_acum_ = 0.f;
    float y2_acum_ = 0.f;
    float inv_misadjustment_ = 0.f;
    int overhang_ = 0.f;
  };

  const Aec3Fft fft_;
  ApmDataDumper* data_dumper_;
  const Aec3Optimization optimization_;
  const EchoCanceller3Config config_;
  const size_t num_capture_channels_;

  std::vector<std::unique_ptr<AdaptiveFirFilter>> refined_filters_;
  std::vector<std::unique_ptr<AdaptiveFirFilter>> coarse_filter_;
  std::vector<std::unique_ptr<RefinedFilterUpdateGain>> refined_gains_;
  std::vector<std::unique_ptr<CoarseFilterUpdateGain>> coarse_gains_;
  std::vector<FilterMisadjustmentEstimator> filter_misadjustment_estimators_;
  std::vector<size_t> poor_coarse_filter_counters_;
  std::vector<std::vector<std::array<float, kFftLengthBy2Plus1>>>
      refined_frequency_responses_;
  std::vector<std::vector<float>> refined_impulse_responses_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(Subtractor);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_SUBTRACTOR_H_
