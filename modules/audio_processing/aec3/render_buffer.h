/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC3_RENDER_BUFFER_H_
#define MODULES_AUDIO_PROCESSING_AEC3_RENDER_BUFFER_H_

#include <stddef.h>

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/block_buffer.h"
#include "modules/audio_processing/aec3/fft_buffer.h"
#include "modules/audio_processing/aec3/fft_data.h"
#include "modules/audio_processing/aec3/spectrum_buffer.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

// Provides a buffer of the render data for the echo remover.
class RenderBuffer {
 public:
  RenderBuffer(aec3::BlockBuffer* block_buffer,
               SpectrumBuffer* spectrum_buffer,
               FftBuffer* fft_buffer);
  ~RenderBuffer();

  // Get a block.
  const std::vector<std::vector<std::vector<float>>>& Block(
      int buffer_offset_blocks) const {
    int position =
        block_buffer_->OffsetIndex(block_buffer_->read, buffer_offset_blocks);
    return block_buffer_->buffer[position];
  }

  // Get the spectrum from one of the FFTs in the buffer.
  const std::vector<std::array<float, kFftLengthBy2Plus1>>& Spectrum(
      int buffer_offset_ffts) const {
    int position = spectrum_buffer_->OffsetIndex(spectrum_buffer_->read,
                                                 buffer_offset_ffts);
    return spectrum_buffer_->buffer[position];
  }

  // Returns the circular fft buffer.
  RTC_VIEW(const std::vector<FftData>) GetFftBuffer() const {
    return fft_buffer_->buffer;
  }

  // Returns the current position in the circular buffer.
  size_t Position() const {
    RTC_DCHECK_EQ(spectrum_buffer_->read, fft_buffer_->read);
    RTC_DCHECK_EQ(spectrum_buffer_->write, fft_buffer_->write);
    return fft_buffer_->read;
  }

  // Returns the sum of the spectrums for a certain number of FFTs.
  void SpectralSum(size_t num_spectra,
                   RTC_VIEW(float) /* kFftLengthBy2Plus1 */ X2) const;

  // Returns the sums of the spectrums for two numbers of FFTs.
  void SpectralSums(size_t num_spectra_shorter,
                    size_t num_spectra_longer,
                    RTC_VIEW(float) /* kFftLengthBy2Plus1 */ X2_shorter,
                    RTC_VIEW(float) /* kFftLengthBy2Plus1 */ X2_longer) const;

  // Gets the recent activity seen in the render signal.
  bool GetRenderActivity() const { return render_activity_; }

  // Specifies the recent activity seen in the render signal.
  void SetRenderActivity(bool activity) { render_activity_ = activity; }

  // Returns the headroom between the write and the read positions in the
  // buffer.
  int Headroom() const {
    // The write and read indices are decreased over time.
    int headroom =
        fft_buffer_->write < fft_buffer_->read
            ? fft_buffer_->read - fft_buffer_->write
            : fft_buffer_->size - fft_buffer_->write + fft_buffer_->read;

    RTC_DCHECK_LE(0, headroom);
    RTC_DCHECK_GE(fft_buffer_->size, headroom);

    return headroom;
  }

  // Returns a reference to the spectrum buffer.
  const SpectrumBuffer& GetSpectrumBuffer() const { return *spectrum_buffer_; }

  // Returns a reference to the block buffer.
  const aec3::BlockBuffer& GetBlockBuffer() const { return *block_buffer_; }

 private:
  const aec3::BlockBuffer* const block_buffer_;
  const SpectrumBuffer* const spectrum_buffer_;
  const FftBuffer* const fft_buffer_;
  bool render_activity_ = false;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(RenderBuffer);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AEC3_RENDER_BUFFER_H_
