/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_CODING_NETEQ_NORMAL_H_
#define MODULES_AUDIO_CODING_NETEQ_NORMAL_H_

#include <stddef.h>  // size_t
#include <stdbool.h>
#include <stdint.h>

#include <vector>

#include "rtc_base/constructor_magic.h"
#include "modules/audio_coding/neteq/audio_multi_vector.h"
// #include "modules/audio_coding/neteq/defines.h"
#include "rtc_base/numerics/safe_conversions.h"

namespace webrtc {

// Forward declarations.
/* class AudioMultiVector;
class BackgroundNoise;
class DecoderDatabase;
class Expand; */

// This class provides the "Normal" DSP operation, that is performed when
// there is no data loss, no need to stretch the timing of the signal, and
// no other "special circumstances" are at hand.
class Normal {
 public:
  Normal(int fs_hz/* ,
         DecoderDatabase* decoder_database,
         const BackgroundNoise& background_noise,
         Expand* expand */)
      : fs_hz_(fs_hz),
        /* decoder_database_(decoder_database),
        background_noise_(background_noise),
        expand_(expand), */
        samples_per_ms_(/* rtc::CheckedDivExact */RTC_CHECK_DIV_EXACT(fs_hz_, 1000)),
        default_win_slope_Q14_(
            rtc::dchecked_cast<uint16_t>((1 << 14) / samples_per_ms_)) {}

  virtual ~Normal() {}

  // Performs the "Normal" operation. The decoder data is supplied in |input|,
  // having |length| samples in total for all channels (interleaved). The
  // result is written to |output|. The number of channels allocated in
  // |output| defines the number of channels that will be used when
  // de-interleaving |input|. |last_mode| contains the mode used in the previous
  // GetAudio call (i.e., not the current one).
  int Process(const int16_t* input,
              size_t length,
              /* Modes last_mode, */
              AudioMultiVector* output);

 private:
  int fs_hz_;
  /* DecoderDatabase* decoder_database_;
  const BackgroundNoise& background_noise_;
  Expand* expand_; */
  const size_t samples_per_ms_;
  const int16_t default_win_slope_Q14_;

  RTC_DISALLOW_COPY_AND_ASSIGN(Normal);
};

}  // namespace webrtc
#endif  // MODULES_AUDIO_CODING_NETEQ_NORMAL_H_
