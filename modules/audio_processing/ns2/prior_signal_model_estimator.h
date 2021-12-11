/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_PRIOR_SIGNAL_MODEL_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_NS_PRIOR_SIGNAL_MODEL_ESTIMATOR_H_

#include "modules/audio_processing/ns2/histograms.h"
#include "modules/audio_processing/ns2/prior_signal_model.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Estimator of the prior signal model parameters.
class PriorSignalModelEstimator {
 public:
  explicit PriorSignalModelEstimator(float lrt_initial_value);

  // Updates the model estimate.
  void Update(const Histograms& h);

  // Returns the estimated model.
  const PriorSignalModel& get_prior_model() const { return prior_model_; }

 private:
  PriorSignalModel prior_model_;

  RTC_DISALLOW_COPY_AND_ASSIGN(PriorSignalModelEstimator);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_PRIOR_SIGNAL_MODEL_ESTIMATOR_H_
