/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_HISTOGRAMS_H_
#define MODULES_AUDIO_PROCESSING_NS_HISTOGRAMS_H_

// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/ns2/ns_common.h"
#include "modules/audio_processing/ns2/signal_model.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

constexpr int kHistogramSize = 1000;

// Class for handling the updating of histograms.
class Histograms {
 public:
  Histograms();

  // Clears the histograms.
  void Clear();

  // Extracts thresholds for feature parameters and updates the corresponding
  // histogram.
  void Update(const SignalModel& features_);

  // Methods for accessing the histograms.
  RTC_VIEW(const int) get_lrt() const {
    return RTC_MAKE_VIEW(const int)(lrt_);
  }
  RTC_VIEW(const int) get_spectral_flatness() const {
    return RTC_MAKE_VIEW(const int)(spectral_flatness_);
  }
  RTC_VIEW(const int) get_spectral_diff() const {
    return RTC_MAKE_VIEW(const int)(spectral_diff_);
  }

 private:
  int lrt_[kHistogramSize];
  int spectral_flatness_[kHistogramSize];
  int spectral_diff_[kHistogramSize];

  RTC_DISALLOW_COPY_AND_ASSIGN(Histograms);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_HISTOGRAMS_H_
