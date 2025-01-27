/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_HIGH_PASS_FILTER_H_
#define MODULES_AUDIO_PROCESSING_HIGH_PASS_FILTER_H_

#include <memory>
#include <vector>

// #include "modules/audio_processing/utility/cascaded_biquad_filter.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

class AudioBuffer;
class CascadedBiQuadFilter;

class HighPassFilter {
 public:
  HighPassFilter(int sample_rate_hz, size_t num_channels);
  ~HighPassFilter();

  void Process(AudioBuffer* audio, bool use_split_band_data);
  void Process(std::vector<std::vector<float>>* audio);
  void Reset();
  void Reset(size_t num_channels);

  int sample_rate_hz() const { return sample_rate_hz_; }
  size_t num_channels() const { return filters_.size(); }

 private:
  const int sample_rate_hz_;
  std::vector<std::unique_ptr<CascadedBiQuadFilter>> filters_;

  RTC_DISALLOW_COPY_AND_ASSIGN(HighPassFilter);
};
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_HIGH_PASS_FILTER_H_
