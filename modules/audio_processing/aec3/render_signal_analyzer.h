/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_RENDER_SIGNAL_ANALYZER_H_
#define MODULES_AUDIO_PROCESSING_AEC3_RENDER_SIGNAL_ANALYZER_H_

#include <algorithm>
// #include <array>
#include <cstddef>

#include "rtc_base/optional.h"
#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/echo_canceller3_config.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/render_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Provides functionality for analyzing the properties of the render signal.
class RenderSignalAnalyzer {
 public:
  explicit RenderSignalAnalyzer(const EchoCanceller3Config& config);
  ~RenderSignalAnalyzer();

  // Updates the render signal analysis with the most recent render signal.
  void Update(const RenderBuffer& render_buffer,
              const rtc::Optional<size_t>& delay_partitions);

  // Returns true if the render signal is poorly exciting.
  bool PoorSignalExcitation() const {
    RTC_DCHECK_LT(2, narrow_band_counters_view_.size());
    return std::any_of(narrow_band_counters_view_.begin(),
                       narrow_band_counters_view_.end(),
                       [](size_t a) { return a > 10; });
  }

  // Zeros the array around regions with narrow bands signal characteristics.
  void MaskRegionsAroundNarrowBands(
      RTC_VIEW(float) /* kFftLengthBy2Plus1 */ v) const;

  rtc::Optional<int> NarrowPeakBand() const { return narrow_peak_band_; }

 private:
  const int strong_peak_freeze_duration_;
  size_t narrow_band_counters_[kFftLengthBy2 - 1];
  RTC_VIEW(size_t) narrow_band_counters_view_ = RTC_MAKE_VIEW(size_t)(narrow_band_counters_);
  rtc::Optional<int> narrow_peak_band_;
  size_t narrow_peak_counter_;

  RTC_DISALLOW_COPY_AND_ASSIGN(RenderSignalAnalyzer);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_RENDER_SIGNAL_ANALYZER_H_
