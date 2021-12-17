/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AUDIO_BUFFER_H_
#define MODULES_AUDIO_PROCESSING_AUDIO_BUFFER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include "modules/include/audio_frame.h"
#include "common_audio/channel_buffer.h"
// #include "modules/audio_processing/include/audio_processing.h"

namespace webrtc {

class PushSincResampler;
class IFChannelBuffer;
class SplittingFilter;

enum Band { kBand0To8kHz = 0, kBand8To16kHz = 1, kBand16To24kHz = 2 };

// Stores any audio data in a way that allows the audio processing module to
// operate on it in a controlled manner.
class AudioBuffer {
 public:
  // TODO(ajm): Switch to take ChannelLayouts.
  static const int kSplitBandSize = 160;
  static const size_t kMaxSampleRate = 384000;

  AudioBuffer(size_t input_rate,
              size_t input_num_channels,
              size_t buffer_rate,
              size_t buffer_num_channels,
              size_t output_rate,
              size_t output_num_channels);

  // The constructor below will be deprecated.
  AudioBuffer(size_t input_num_frames,
              size_t input_num_channels,
              size_t buffer_num_frames,
              size_t buffer_num_channels,
              size_t output_num_frames);
  virtual ~AudioBuffer();

  // Specify that downmixing should be done by selecting a single channel.
  void set_downmixing_to_specific_channel(size_t channel);

  // Specify that downmixing should be done by averaging all channels,.
  void set_downmixing_by_averaging();

  // Set the number of channels in the buffer. The specified number of channels
  // cannot be larger than the specified buffer_num_channels. The number is also
  // reset at each call to CopyFrom or InterleaveFrom.
  void set_num_channels(size_t num_channels);

  size_t num_channels() const { return num_channels_; }
  size_t num_frames() const { return buffer_num_frames_; }
  size_t num_frames_per_band() const { return num_split_frames_; }
  size_t num_bands() const { return num_bands_; }

  // Returns a pointer array to the full-band channels.
  // Usage:
  // channels()[channel][sample].
  // Where:
  // 0 <= channel < |buffer_num_channels_|
  // 0 <= sample < |buffer_num_frames_|
  int16_t* const* channels() { mixed_low_pass_valid_ = false; return data_->ibuf()->channels(); }
  const int16_t* const* channels_const() const { return data_->ibuf_const()->channels(); }

  // Returns a pointer array to the bands for a specific channel.
  // Usage:
  // split_bands(channel)[band][sample].
  // Where:
  // 0 <= channel < |buffer_num_channels_|
  // 0 <= band < |num_bands_|
  // 0 <= sample < |num_split_frames_|
  int16_t* const* split_bands(size_t channel) {
    mixed_low_pass_valid_ = false;
    return split_data_.get() ? split_data_->ibuf()->bands(channel)
                             : data_->ibuf()->bands(channel);
  }
  const int16_t* const* split_bands_const(size_t channel) const {
    return split_data_.get() ? split_data_->ibuf_const()->bands(channel)
                             : data_->ibuf_const()->bands(channel);
  }

  // Returns a pointer array to the channels for a specific band.
  // Usage:
  // split_channels(band)[channel][sample].
  // Where:
  // 0 <= band < |num_bands_|
  // 0 <= channel < |buffer_num_channels_|
  // 0 <= sample < |num_split_frames_|
  int16_t* const* split_channels(Band band) {
    mixed_low_pass_valid_ = false;
    if (split_data_.get()) {
      return split_data_->ibuf()->channels(band);
    } else {
      return band == kBand0To8kHz ? data_->ibuf()->channels() : nullptr;
    }
  }
  const int16_t* const* split_channels_const(Band band) const {
    if (split_data_.get()) {
      return split_data_->ibuf_const()->channels(band);
    } else {
      return band == kBand0To8kHz ? data_->ibuf_const()->channels() : nullptr;
    }
  }

  // Returns a pointer to the ChannelBuffer that encapsulates the full-band
  // data.
  ChannelBuffer<int16_t>* data() { mixed_low_pass_valid_ = false; return data_->ibuf(); }
  const ChannelBuffer<int16_t>* data() const { return data_->ibuf_const(); }

  // Returns a pointer to the ChannelBuffer that encapsulates the split data.
  ChannelBuffer<int16_t>* split_data() {
    mixed_low_pass_valid_ = false;
    return split_data_.get() ? split_data_->ibuf() : data_->ibuf();
  }
  const ChannelBuffer<int16_t>* split_data() const {
    return split_data_.get() ? split_data_->ibuf_const() : data_->ibuf_const();
  }

  // Returns a pointer to the low-pass data downmixed to mono. If this data
  // isn't already available it re-calculates it.
  const int16_t* mixed_low_pass_data();
  const int16_t* low_pass_reference(int channel) const;

  void set_activity(AudioFrame::VADActivity activity);
  AudioFrame::VADActivity activity() const;

  // Use for int16 interleaved data.
  void DeinterleaveFrom(AudioFrame* audioFrame);
  // If |data_changed| is false, only the non-audio data members will be copied
  // to |frame|.
  void InterleaveTo(AudioFrame* frame, bool data_changed) const;

  // Use for float deinterleaved data.
  /* void CopyFrom(const float* const* data, const StreamConfig& stream_config);
  void CopyTo(const StreamConfig& stream_config, float* const* data);
  void CopyLowPassToReference(); */

  // Splits the buffer data into frequency bands.
  void SplitIntoFrequencyBands();

  // Recombines the frequency bands into a full-band signal.
  void MergeFrequencyBands();

  // Copies the split bands data into the integer two-dimensional array.
  void ExportSplitChannelData(size_t channel,
                              int16_t* const* split_band_data) const;

  // Copies the data in the integer two-dimensional array into the split_bands
  // data.
  void ImportSplitChannelData(size_t channel,
                              const int16_t* const* split_band_data);

  static const size_t kMaxSplitFrameLength = 160;
  static const size_t kMaxNumBands = 3;

  // Deprecated methods, will be removed soon.
  float* const* channels_f() { mixed_low_pass_valid_ = false; return data_->fbuf()->channels(); }
  const float* const* channels_const_f() const { return data_->fbuf_const()->channels(); }
  float* const* split_bands_f(size_t channel) {
    mixed_low_pass_valid_ = false;
    return split_data_.get() ? split_data_->fbuf()->bands(channel)
                             : data_->fbuf()->bands(channel);
  }
  const float* const* split_bands_const_f(size_t channel) const {
    return split_data_.get() ? split_data_->fbuf_const()->bands(channel)
                             : data_->fbuf_const()->bands(channel);
  }
  float* const* split_channels_f(Band band) {
    mixed_low_pass_valid_ = false;
    if (split_data_.get()) {
      return split_data_->fbuf()->channels(band);
    } else {
      return band == kBand0To8kHz ? data_->fbuf()->channels() : nullptr;
    }
  }
  const float* const* split_channels_const_f(Band band) const {
    if (split_data_.get()) {
      return split_data_->fbuf_const()->channels(band);
    } else {
      return band == kBand0To8kHz ? data_->fbuf_const()->channels() : nullptr;
    }
  }
  ChannelBuffer<float>* data_f() { mixed_low_pass_valid_ = false; return data_->fbuf(); }
  const ChannelBuffer<float>* data_f() const { return data_->fbuf_const(); }
  ChannelBuffer<float>* split_data_f() {
    mixed_low_pass_valid_ = false;
    return split_data_.get() ? split_data_->fbuf() : data_->fbuf();
  }
  const ChannelBuffer<float>* split_data_f() const {
    return split_data_.get() ? split_data_->fbuf_const() : data_->fbuf_const();
  }

 private:
  /* FRIEND_TEST_ALL_PREFIXES(AudioBufferTest,
                           SetNumChannelsSetsChannelBuffersNumChannels); */
  // Called from DeinterleaveFrom() and CopyFrom().
  void RestoreNumChannels();

  // The audio is passed into DeinterleaveFrom() or CopyFrom() with input
  // format (samples per channel and number of channels).
  const size_t input_num_frames_;
  const size_t input_num_channels_;
  // The audio is stored by DeinterleaveFrom() or CopyFrom() with processing
  // format.
  const size_t buffer_num_frames_;
  const size_t buffer_num_channels_;
  // The audio is returned by InterleaveTo() and CopyTo() with output samples
  // per channels and the current number of channels. This last one can be
  // changed at any time using set_num_channels().
  const size_t output_num_frames_;
  const size_t output_num_channels_;

  size_t num_channels_;
  size_t num_bands_;
  size_t num_split_frames_;
  bool mixed_low_pass_valid_;
  bool reference_copied_;
  AudioFrame::VADActivity activity_;

  std::unique_ptr<IFChannelBuffer> data_;
  std::unique_ptr<IFChannelBuffer> split_data_;
  std::unique_ptr<SplittingFilter> splitting_filter_;
  std::unique_ptr<ChannelBuffer<int16_t>> mixed_low_pass_channels_;
  std::unique_ptr<ChannelBuffer<int16_t>> low_pass_reference_channels_;
  std::unique_ptr<IFChannelBuffer> input_buffer_;
  std::unique_ptr<IFChannelBuffer> output_buffer_;
  std::unique_ptr<ChannelBuffer<float>> process_buffer_;
  std::vector<std::unique_ptr<PushSincResampler>> input_resamplers_;
  std::vector<std::unique_ptr<PushSincResampler>> output_resamplers_;
  bool downmix_by_averaging_ = true;
  size_t channel_for_downmixing_ = 0;
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AUDIO_BUFFER_H_
