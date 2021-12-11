/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/ns2/signal_model.h"
#include "rtc_base/view.h"

namespace webrtc {

SignalModel::SignalModel() {
  constexpr float kSfFeatureThr = 0.5f;

  lrt = kLtrFeatureThr;
  spectral_flatness = kSfFeatureThr;
  spectral_diff = kSfFeatureThr;

  // avg_log_lrt.fill(kLtrFeatureThr);
  RTC_VIEW(float) avg_log_lrt_view = RTC_MAKE_VIEW(float)(avg_log_lrt);
  avg_log_lrt_view.fill(kLtrFeatureThr);
}

}  // namespace webrtc
