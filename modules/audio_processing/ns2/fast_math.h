/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_NS_FAST_MATH_H_
#define MODULES_AUDIO_PROCESSING_NS_FAST_MATH_H_

#include "rtc_base/view.h"

namespace webrtc {

// Sqrt approximation.
float SqrtFastApproximation(float f);

// Log base conversion log(x) = log2(x)/log2(e).
float LogApproximation(float x);
void LogApproximation(RTC_VIEW(const float) x, RTC_VIEW(float) y);

// 2^x approximation.
float Pow2Approximation(float p);

// x^p approximation.
float PowApproximation(float x, float p);

// e^x approximation.
float ExpApproximation(float x);
void ExpApproximation(RTC_VIEW(const float) x, RTC_VIEW(float) y);
void ExpApproximationSignFlip(RTC_VIEW(const float) x,
                              RTC_VIEW(float) y);
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_NS_FAST_MATH_H_
