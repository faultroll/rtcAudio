/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_ERLE_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_AEC3_ERLE_ESTIMATOR_H_

#include <stddef.h>

// #include <array>
#include <memory>
#include <vector>

#include "rtc_base/optional.h"
#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/fullband_erle_estimator.h"
#include "modules/audio_processing/aec3/render_buffer.h"
#include "modules/audio_processing/aec3/signal_dependent_erle_estimator.h"
#include "modules/audio_processing/aec3/subband_erle_estimator.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Estimates the echo return loss enhancement. One estimate is done per subband
// and another one is done using the aggreation of energy over all the subbands.
class ErleEstimator {
 public:
  ErleEstimator(size_t startup_phase_length_blocks,
                const EchoCanceller3Config& config,
                size_t num_capture_channels);
  ~ErleEstimator();

  // Resets the fullband ERLE estimator and the subbands ERLE estimators.
  void Reset(bool delay_change);

  // Updates the ERLE estimates.
  void Update(
      const RenderBuffer& render_buffer,
      const std::vector<std::vector<std::array<float, kFftLengthBy2Plus1>>>&
          filter_frequency_responses,
      RTC_VIEW(const float) /* kFftLengthBy2Plus1 */ avg_render_spectrum_with_reverb,
      const std::vector<std::array<float, kFftLengthBy2Plus1>>& capture_spectra,
      const std::vector<std::array<float, kFftLengthBy2Plus1>>& subtractor_spectra,
      const std::vector<bool>& converged_filters);

  // Returns the most recent subband ERLE estimates.
  const std::vector<std::array<float, kFftLengthBy2Plus1>>& Erle() const {
    return signal_dependent_erle_estimator_
               ? signal_dependent_erle_estimator_->Erle()
               : subband_erle_estimator_.Erle();
  }

  // Returns the subband ERLE that are estimated during onsets (only used for
  // testing).
  const std::vector<std::array<float, kFftLengthBy2Plus1>>& ErleOnsets()
      const {
    return subband_erle_estimator_.ErleOnsets();
  }

  // Returns the fullband ERLE estimate.
  float FullbandErleLog2() const {
    return fullband_erle_estimator_.FullbandErleLog2();
  }

  // Returns an estimation of the current linear filter quality based on the
  // current and past fullband ERLE estimates. The returned value is a float
  // vector with content between 0 and 1 where 1 indicates that, at this current
  // time instant, the linear filter is reaching its maximum subtraction
  // performance.
  RTC_VIEW(const rtc::Optional<float>) GetInstLinearQualityEstimates()
      const {
    return fullband_erle_estimator_.GetInstLinearQualityEstimates();
  }

  void Dump(const std::unique_ptr<ApmDataDumper>& data_dumper) const;

 private:
  const size_t startup_phase_length_blocks_;
  FullBandErleEstimator fullband_erle_estimator_;
  SubbandErleEstimator subband_erle_estimator_;
  std::unique_ptr<SignalDependentErleEstimator> signal_dependent_erle_estimator_;
  size_t blocks_since_reset_ = 0;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_ERLE_ESTIMATOR_H_
