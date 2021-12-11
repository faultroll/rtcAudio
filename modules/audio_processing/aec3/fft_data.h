/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_FFT_DATA_H_
#define MODULES_AUDIO_PROCESSING_AEC3_FFT_DATA_H_

// Defines WEBRTC_ARCH_X86_FAMILY, used below.
#include "rtc_base/system/arch.h"

#if defined(WEBRTC_ARCH_X86_FAMILY)
#include <emmintrin.h>
#endif
#include <algorithm>
// #include <array>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"

namespace webrtc {

// Struct that holds imaginary data produced from 128 point real-valued FFTs.
struct FftData {
  // Copies the data in src.
  void Assign(const FftData& src) {
    std::copy(src.re_view.begin(), src.re_view.end(), re_view.begin());
    std::copy(src.im_view.begin(), src.im_view.end(), im_view.begin());
    im_view[0] = im_view[kFftLengthBy2] = 0;
  }

  // Clears all the imaginary.
  void Clear() {
    re_view.fill(0.f);
    im_view.fill(0.f);
  }

  // Computes the power spectrum of the data.
  void SpectrumAVX2(RTC_VIEW(float) power_spectrum) const;

  // Computes the power spectrum of the data.
  void Spectrum(Aec3Optimization optimization,
                RTC_VIEW(float) power_spectrum) const {
    RTC_DCHECK_EQ(kFftLengthBy2Plus1, power_spectrum.size());
    switch (optimization) {
#if defined(WEBRTC_ARCH_X86_FAMILY)
      case Aec3Optimization::kSse2: {
        constexpr int kNumFourBinBands = kFftLengthBy2 / 4;
        constexpr int kLimit = kNumFourBinBands * 4;
        for (size_t k = 0; k < kLimit; k += 4) {
          const __m128 r = _mm_loadu_ps(&re_view[k]);
          const __m128 i = _mm_loadu_ps(&im_view[k]);
          const __m128 ii = _mm_mul_ps(i, i);
          const __m128 rr = _mm_mul_ps(r, r);
          const __m128 rrii = _mm_add_ps(rr, ii);
          _mm_storeu_ps(&power_spectrum[k], rrii);
        }
        power_spectrum[kFftLengthBy2] = re_view[kFftLengthBy2] * re_view[kFftLengthBy2] +
                                        im_view[kFftLengthBy2] * im_view[kFftLengthBy2];
      } break;
      case Aec3Optimization::kAvx2:
        SpectrumAVX2(power_spectrum);
        break;
#endif
      default:
        std::transform(re_view.begin(), re_view.end(), im_view.begin(), power_spectrum.begin(),
                       [](float a, float b) { return a * a + b * b; });
    }
  }

  // Copy the data from an interleaved array.
  void CopyFromPackedArray(RTC_VIEW(const float) /* kFftLength */ v) {
    re_view[0] = v[0];
    re_view[kFftLengthBy2] = v[1];
    im_view[0] = im_view[kFftLengthBy2] = 0;
    for (size_t k = 1, j = 2; k < kFftLengthBy2; ++k) {
      re_view[k] = v[j++];
      im_view[k] = v[j++];
    }
  }

  // Copies the data into an interleaved array.
  void CopyToPackedArray(RTC_VIEW(float) /* kFftLength */ v) const {
    // RTC_DCHECK(v);
    v[0] = re_view[0];
    v[1] = re_view[kFftLengthBy2];
    for (size_t k = 1, j = 2; k < kFftLengthBy2; ++k) {
      v[j++] = re_view[k];
      v[j++] = im_view[k];
    }
  }

  float re[kFftLengthBy2Plus1];
  RTC_VIEW(float) re_view = RTC_MAKE_VIEW(float)(re);
  float im[kFftLengthBy2Plus1];
  RTC_VIEW(float) im_view = RTC_MAKE_VIEW(float)(im);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_FFT_DATA_H_
