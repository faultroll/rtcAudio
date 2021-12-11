/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_SUPPRESSION_PARAMS_H_
#define MODULES_AUDIO_PROCESSING_NS_SUPPRESSION_PARAMS_H_

#include "modules/audio_processing/ns2/ns_config.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

struct SuppressionParams {
  explicit SuppressionParams(NsConfig::SuppressionLevel suppression_level);

  float over_subtraction_factor;
  float minimum_attenuating_gain;
  bool use_attenuation_adjustment;

  RTC_DISALLOW_COPY_AND_ASSIGN(SuppressionParams);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_SUPPRESSION_PARAMS_H_
