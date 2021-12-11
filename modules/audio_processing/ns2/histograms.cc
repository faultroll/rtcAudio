/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/ns2/histograms.h"

namespace webrtc {

Histograms::Histograms() {
  Clear();
}

void Histograms::Clear() {
  // lrt_.fill(0);
  // spectral_flatness_.fill(0);
  // spectral_diff_.fill(0);

  RTC_VIEW(int) lrt_view = RTC_MAKE_VIEW(int)(lrt_);
  lrt_view.fill(0);
  RTC_VIEW(int) spectral_flatness_view = RTC_MAKE_VIEW(int)(spectral_flatness_);
  spectral_flatness_view.fill(0);
  RTC_VIEW(int) spectral_diff_view = RTC_MAKE_VIEW(int)(spectral_diff_);
  spectral_diff_view.fill(0);
}

void Histograms::Update(const SignalModel& features_) {
  // Update the histogram for the LRT.
  constexpr float kOneByBinSizeLrt = 1.f / kBinSizeLrt;
  if (features_.lrt < kHistogramSize * kBinSizeLrt &&
      features_.lrt >= 0.f) {
    ++lrt_[(int)(kOneByBinSizeLrt * features_.lrt)];
  }

  // Update histogram for the spectral flatness.
  constexpr float kOneByBinSizeSpecFlat = 1.f / kBinSizeSpecFlat;
  if (features_.spectral_flatness < kHistogramSize * kBinSizeSpecFlat &&
      features_.spectral_flatness >= 0.f) {
    ++spectral_flatness_[(int)(kOneByBinSizeSpecFlat * features_.spectral_flatness)];
  }

  // Update histogram for the spectral difference.
  constexpr float kOneByBinSizeSpecDiff = 1.f / kBinSizeSpecDiff;
  if (features_.spectral_diff < kHistogramSize * kBinSizeSpecDiff &&
      features_.spectral_diff >= 0.f) {
    ++spectral_diff_[(int)(kOneByBinSizeSpecDiff * features_.spectral_diff)];
  }
}

}  // namespace webrtc
