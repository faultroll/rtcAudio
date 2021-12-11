/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_DOWN_SAMPLER_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_DOWN_SAMPLER_H_

#include "rtc_base/view.h"
#include "rtc_base/constructor_magic.h"
#include "modules/audio_processing/agc2/biquad_filter.h"

namespace webrtc {

class ApmDataDumper;

class DownSampler {
 public:
  explicit DownSampler(ApmDataDumper* data_dumper);
  void Initialize(int sample_rate_hz);

  void DownSample(RTC_VIEW(const float) in, RTC_VIEW(float) out);

 private:
  ApmDataDumper* data_dumper_;
  int sample_rate_hz_;
  int down_sampling_factor_;
  BiQuadFilter low_pass_filter_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(DownSampler);
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_LEVEL_CONTROLLER_DOWN_SAMPLER_H_
