/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_UTILITY_PFFFT_WRAPPER_H_
#define MODULES_AUDIO_PROCESSING_UTILITY_PFFFT_WRAPPER_H_

#include <memory>

#include "rtc_base/view.h"
#include "rtc_base/constructor_magic.h"

// Forward declaration.
struct PFFFT_Setup;

namespace webrtc {

// Pretty-Fast Fast Fourier Transform (PFFFT) wrapper class.
// Not thread safe.
class Pffft {
 public:
  enum class FftType { kReal, kComplex };

  // 1D floating point buffer used as input/output data type for the FFT ops.
  // It must be constructed using Pffft::CreateBuffer().
  class FloatBuffer {
   public:
    ~FloatBuffer();

    RTC_VIEW(const float) GetConstView() const;
    RTC_VIEW(float) GetView();

   private:
    friend class Pffft;
    FloatBuffer(size_t fft_size, FftType fft_type);
    const float* const_data() const { return data_; }
    float* data() { return data_; }
    size_t size() const { return size_; }

    const size_t size_;
    float* const data_;

    RTC_DISALLOW_COPY_AND_ASSIGN(FloatBuffer);
  };

  // TODO(https://crbug.com/webrtc/9577): Consider adding a factory and making
  // the ctor private.
  // static std::unique_ptr<Pffft> Create(size_t fft_size,
  // FftType fft_type); Ctor. |fft_size| must be a supported size (see
  // Pffft::IsValidFftSize()). If not supported, the code will crash.
  Pffft(size_t fft_size, FftType fft_type);
  ~Pffft();

  // Returns true if the FFT size is supported.
  static bool IsValidFftSize(size_t fft_size, FftType fft_type);

  // Returns true if SIMD code optimizations are being used.
  static bool IsSimdEnabled();

  // Creates a buffer of the right size.
  std::unique_ptr<FloatBuffer> CreateBuffer() const;

  // TODO(https://crbug.com/webrtc/9577): Overload with rtc::ArrayView args.
  // Computes the forward fast Fourier transform.
  void ForwardTransform(const FloatBuffer& in, FloatBuffer* out, bool ordered);
  // Computes the backward fast Fourier transform.
  void BackwardTransform(const FloatBuffer& in, FloatBuffer* out, bool ordered);

  // Multiplies the frequency components of |fft_x| and |fft_y| and accumulates
  // them into |out|. The arrays must have been obtained with
  // ForwardTransform(..., /*ordered=*/false) - i.e., |fft_x| and |fft_y| must
  // not be ordered.
  void FrequencyDomainConvolve(const FloatBuffer& fft_x,
                               const FloatBuffer& fft_y,
                               FloatBuffer* out,
                               float scaling = 1.f);

 private:
  const size_t fft_size_;
  const FftType fft_type_;
  PFFFT_Setup* pffft_status_;
  float* const scratch_buffer_;

  RTC_DISALLOW_COPY_AND_ASSIGN(Pffft);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_UTILITY_PFFFT_WRAPPER_H_
