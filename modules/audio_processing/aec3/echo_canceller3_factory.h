/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_AUDIO_ECHO_CANCELLER3_FACTORY_H_
#define API_AUDIO_ECHO_CANCELLER3_FACTORY_H_

#include <memory>

#include "modules/audio_processing/aec3/echo_canceller3_config.h"
#include "modules/audio_processing/include/echo_control.h"

namespace webrtc {

class EchoCanceller3Factory : public EchoControlFactory {
 public:
  // Factory producing EchoCanceller3 instances with the default configuration.
  EchoCanceller3Factory();

  // Factory producing EchoCanceller3 instances with the specified
  // configuration.
  explicit EchoCanceller3Factory(const EchoCanceller3Config& config);

  // Creates an EchoCanceller3 running at the specified sampling rate.
  std::unique_ptr<EchoControl> Create(int sample_rate_hz,
                                      int num_render_channels,
                                      int num_capture_channels) override;

 private:
  const EchoCanceller3Config config_;
};
}  // namespace webrtc

#endif  // API_AUDIO_ECHO_CANCELLER3_FACTORY_H_