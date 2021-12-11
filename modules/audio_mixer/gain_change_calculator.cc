/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_mixer/gain_change_calculator.h"

#include <math.h>

#include <cstdlib>

#include "rtc_base/checks.h"

namespace webrtc {

namespace {
constexpr int16_t kReliabilityThreshold = 100;
}  // namespace

float GainChangeCalculator::CalculateGainChange(
    RTC_VIEW(const int16_t) in,
    RTC_VIEW(const int16_t) out) {
  RTC_DCHECK_EQ(in.size(), out.size());

  std::vector<float> gain(in.size());
  CalculateGain(in, out, gain);
  return CalculateDifferences(gain);
}

float GainChangeCalculator::LatestGain() const {
  return last_reliable_gain_;
}

void GainChangeCalculator::CalculateGain(RTC_VIEW(const int16_t) in,
                                         RTC_VIEW(const int16_t) out,
                                         std::vector<float>& gain) {
  RTC_DCHECK_EQ(in.size(), out.size());
  RTC_DCHECK_EQ(in.size(), gain.size());

  for (size_t i = 0; i < in.size(); ++i) {
    if (std::abs(in.data()[i]) >= kReliabilityThreshold) {
      last_reliable_gain_ = out.data()[i] / static_cast<float>(in.data()[i]);
    }
    gain[i] = last_reliable_gain_;
  }
}

float GainChangeCalculator::CalculateDifferences(
    std::vector<float>& values) {
  float res = 0;
  for (float f : values) {
    res += fabs(f - last_value_);
    last_value_ = f;
  }
  return res;
}
}  // namespace webrtc
