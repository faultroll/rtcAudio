/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_PITCH_SEARCH_INTERNAL_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_PITCH_SEARCH_INTERNAL_H_

#include <stddef.h>

// #include <array>

#include "rtc_base/view.h"
#include "common_audio/real_fourier.h"
#include "modules/audio_processing/agc2/rnn_vad/common.h"
#include "modules/audio_processing/agc2/rnn_vad/pitch_info.h"

namespace webrtc {
namespace rnn_vad {

// Performs 2x decimation without any anti-aliasing filter.
void Decimate2x(RTC_VIEW(const float) /* kBufSize24kHz */ src,
                RTC_VIEW(float) /* kBufSize12kHz */ dst);

// Computes a gain threshold for a candidate pitch period given the initial and
// the previous pitch period and gain estimates and the pitch period ratio used
// to derive the candidate pitch period from the initial period.
float ComputePitchGainThreshold(int candidate_pitch_period,
                                int pitch_period_ratio,
                                int initial_pitch_period,
                                float initial_pitch_gain,
                                int prev_pitch_period,
                                float prev_pitch_gain);

// Computes the sum of squared samples for every sliding frame in the pitch
// buffer. |yy_values| indexes are lags.
//
// The pitch buffer is structured as depicted below:
// |.........|...........|
//      a          b
// The part on the left, named "a" contains the oldest samples, whereas "b" the
// most recent ones. The size of "a" corresponds to the maximum pitch period,
// that of "b" to the frame size (e.g., 16 ms and 20 ms respectively).
void ComputeSlidingFrameSquareEnergies(
    RTC_VIEW(const float) /* kBufSize24kHz */ pitch_buf,
    RTC_VIEW(float) /* kMaxPitch24kHz + 1 */ yy_values);

// Computes the auto-correlation coefficients for a given pitch interval.
// |auto_corr| indexes are inverted lags.
//
// The auto-correlations coefficients are computed as follows:
// |.........|...........|  <- pitch buffer
//           [ x (fixed) ]
// [   y_0   ]
//         [ y_{m-1} ]
// x and y are sub-array of equal length; x is never moved, whereas y slides.
// The cross-correlation between y_0 and x corresponds to the auto-correlation
// for the maximum pitch period. Hence, the first value in |auto_corr| has an
// inverted lag equal to 0 that corresponds to a lag equal to the maximum pitch
// period.
// void ComputePitchAutoCorrelation(
//     RTC_VIEW(const float) /* kBufSize12kHz */ pitch_buf,
//     size_t max_pitch_period,
//     RTC_VIEW(float) /* kNumInvertedLags12kHz */ auto_corr,
//     webrtc::RealFourier* fft);

// Given the auto-correlation coefficients stored according to
// ComputePitchAutoCorrelation() (i.e., using inverted lags), returns the best
// and the second best pitch periods.
std::array<size_t, 2> FindBestPitchPeriods(
    RTC_VIEW(const float) auto_corr,
    RTC_VIEW(const float) pitch_buf,
    size_t max_pitch_period);

// Refines the pitch period estimation given the pitch buffer |pitch_buf| and
// the initial pitch period estimation |inv_lags|. Returns an inverted lag at
// 48 kHz.
size_t RefinePitchPeriod48kHz(
    RTC_VIEW(const float) /* kBufSize24kHz */ pitch_buf,
    RTC_VIEW(const size_t) /* 2 */ inv_lags);

// Refines the pitch period estimation and compute the pitch gain. Returns the
// refined pitch estimation data at 48 kHz.
PitchInfo CheckLowerPitchPeriodsAndComputePitchGain(
    RTC_VIEW(const float) /* kBufSize24kHz */ pitch_buf,
    int initial_pitch_period_48kHz,
    PitchInfo prev_pitch_48kHz);

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_PITCH_SEARCH_INTERNAL_H_
