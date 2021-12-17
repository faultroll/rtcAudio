/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/audio_buffer.h"

#include "common_audio/channel_buffer.h"
#include "common_audio/include/audio_util.h"
#include "common_audio/resampler/push_sinc_resampler.h"
#include "common_audio/signal_processing/include/signal_processing_library.h"
#include "modules/audio_processing/splitting_filter.h"
// #include "modules/audio_processing/common.h"
#include "rtc_base/checks.h"

namespace webrtc {
namespace {

const size_t kSamplesPer16kHzChannel = 160;
const size_t kSamplesPer32kHzChannel = 320;
const size_t kSamplesPer48kHzChannel = 480;
const size_t kMaxSamplesPerChannel = AudioBuffer::kMaxSampleRate / 100;

size_t NumBandsFromFramesPerChannel(size_t num_frames) {
  if (num_frames == kSamplesPer32kHzChannel) {
    return 2;
  }
  if (num_frames == kSamplesPer48kHzChannel) {
    return 3;
  }
  return 1;
}

}  // namespace

AudioBuffer::AudioBuffer(size_t input_rate,
                         size_t input_num_channels,
                         size_t buffer_rate,
                         size_t buffer_num_channels,
                         size_t output_rate,
                         size_t output_num_channels)
    : AudioBuffer(static_cast<int>(input_rate) / 100,
                  input_num_channels,
                  static_cast<int>(buffer_rate) / 100,
                  buffer_num_channels,
                  static_cast<int>(output_rate) / 100) {}

AudioBuffer::AudioBuffer(size_t input_num_frames,
                         size_t input_num_channels,
                         size_t buffer_num_frames,
                         size_t buffer_num_channels,
                         size_t output_num_frames)
    : input_num_frames_(input_num_frames),
      input_num_channels_(input_num_channels),
      buffer_num_frames_(buffer_num_frames),
      buffer_num_channels_(buffer_num_channels),
      output_num_frames_(output_num_frames),
      output_num_channels_(0),
      num_channels_(buffer_num_channels),
      num_bands_(NumBandsFromFramesPerChannel(buffer_num_frames_)),
      num_split_frames_(/* rtc::CheckedDivExact */RTC_CHECK_DIV_EXACT(buffer_num_frames_, num_bands_)),
      mixed_low_pass_valid_(false),
      reference_copied_(false),
      activity_(AudioFrame::kVadUnknown),
      data_(new IFChannelBuffer(buffer_num_frames_, buffer_num_channels_)),
      output_buffer_(new IFChannelBuffer(output_num_frames_, num_channels_)) {
  RTC_DCHECK_GT(input_num_frames_, 0);
  RTC_DCHECK_GT(buffer_num_frames_, 0);
  RTC_DCHECK_GT(output_num_frames_, 0);
  RTC_DCHECK_GT(input_num_channels_, 0);
  RTC_DCHECK_GT(buffer_num_channels_, 0);
  RTC_DCHECK_LE(buffer_num_channels_, input_num_channels_);

  if (input_num_frames_ != buffer_num_frames_ ||
      output_num_frames_ != buffer_num_frames_) {
    // Create an intermediate buffer for resampling.
    process_buffer_.reset(
        new ChannelBuffer<float>(buffer_num_frames_, buffer_num_channels_));

    if (input_num_frames_ != buffer_num_frames_) {
      for (size_t i = 0; i < buffer_num_channels_; ++i) {
        input_resamplers_.push_back(std::unique_ptr<PushSincResampler>(
            new PushSincResampler(input_num_frames_, buffer_num_frames_)));
      }
    }

    if (output_num_frames_ != buffer_num_frames_) {
      for (size_t i = 0; i < buffer_num_channels_; ++i) {
        output_resamplers_.push_back(std::unique_ptr<PushSincResampler>(
            new PushSincResampler(buffer_num_frames_, output_num_frames_)));
      }
    }
  }

  if (num_bands_ > 1) {
    split_data_.reset(
        new IFChannelBuffer(buffer_num_frames_, buffer_num_channels_, num_bands_));
    splitting_filter_.reset(
        new SplittingFilter(buffer_num_channels_, num_bands_, buffer_num_frames_));
  }
}

AudioBuffer::~AudioBuffer() {}

void AudioBuffer::set_downmixing_to_specific_channel(size_t channel) {
  downmix_by_averaging_ = false;
  RTC_DCHECK_GT(input_num_channels_, channel);
  channel_for_downmixing_ = std::min(channel, input_num_channels_ - 1);
}

void AudioBuffer::set_downmixing_by_averaging() {
  downmix_by_averaging_ = true;
}

/* void AudioBuffer::CopyFrom(const float* const* stacked_data,
                           const StreamConfig& stream_config) {
  RTC_DCHECK_EQ(stream_config.num_frames(), input_num_frames_);
  RTC_DCHECK_EQ(stream_config.num_channels(), input_num_channels_);
  const bool downmix_needed = input_num_channels_ > 1 && num_channels_ == 1;
  const bool resampling_needed = input_num_frames_ != buffer_num_frames_;

  // Initialized lazily because there's a different condition in
  // DeinterleaveFrom.
  RestoreNumChannels();
  if (downmix_needed && !input_buffer_) {
    input_buffer_.reset(
        new IFChannelBuffer(input_num_frames_, buffer_num_channels_));
  }

  // Downmix.
  const float* const* data_ptr = stacked_data;
  if (downmix_needed) {
    RTC_DCHECK_GE(kMaxSamplesPerChannel, input_num_frames_);
    DownmixToMono<float, float>(stacked_data, input_num_frames_, input_num_channels_,
                                input_buffer_->fbuf()->channels()[0]);
    data_ptr = input_buffer_->fbuf_const()->channels();
    // float downmix[kMaxSamplesPerChannel];
    // if (downmix_by_averaging_) {
    //   const float kOneByNumChannels = 1.f / input_num_channels_;
    //   for (size_t i = 0; i < input_num_frames_; ++i) {
    //     float value = stacked_data[0][i];
    //     for (size_t j = 1; j < input_num_channels_; ++j) {
    //       value += stacked_data[j][i];
    //     }
    //     downmix[i] = value * kOneByNumChannels;
    //   }
    // }
    // const float* downmixed_data = downmix_by_averaging_
    //                                   ? downmix.data()
    //                                   : stacked_data[channel_for_downmixing_];
  }

  // Resample.
  if (resampling_needed) {
    for (size_t i = 0; i < buffer_num_channels_; ++i) {
      input_resamplers_[i]->Resample(data_ptr[i], input_num_frames_,
                                     process_buffer_->channels()[i],
                                     buffer_num_frames_);
    }
    data_ptr = process_buffer_->channels();
  }

  // Convert to the S16 range.
  for (size_t i = 0; i < buffer_num_channels_; ++i) {
    FloatToFloatS16(data_ptr[i], buffer_num_frames_,
                    data_->fbuf()->channels()[i]);
  }
}

void AudioBuffer::CopyTo(const StreamConfig& stream_config,
                         float* const* stacked_data) {
  RTC_DCHECK_EQ(stream_config.num_frames(), output_num_frames_);
  RTC_DCHECK(stream_config.num_channels() == num_channels_ ||
             num_channels_ == 1);
  output_num_channels_ = stream_config.num_channels();
  // const bool upmix_needed = output_num_channels_ > 1 || num_channels_ == 1;
  const bool resampling_needed = output_num_frames_ != buffer_num_frames_;

  // Convert to the float range.
  float* const* data_ptr = stacked_data;
  if (resampling_needed) {
    // Convert to an intermediate buffer for subsequent resampling.
    data_ptr = process_buffer_->channels();
  }
  for (size_t i = 0; i < num_channels_; ++i) {
    FloatS16ToFloat(data_->fbuf()->channels()[i], buffer_num_frames_,
                    data_ptr[i]);
  }

  // Resample.
  if (resampling_needed) {
    for (size_t i = 0; i < num_channels_; ++i) {
      output_resamplers_[i]->Resample(data_ptr[i], buffer_num_frames_, stacked_data[i],
                                      output_num_frames_);
    }
  }

  // Upmix.
  for (size_t i = num_channels_; i < output_num_channels_; ++i) {
    // UpmixMonoToInterleaved
    memcpy(stacked_data[i], stacked_data[0], output_num_frames_ * sizeof(**stacked_data));
  }
} */

const int16_t* AudioBuffer::mixed_low_pass_data() {
  if (buffer_num_channels_ == 1) {
    return split_bands_const(0)[kBand0To8kHz];
  }

  if (!mixed_low_pass_valid_) {
    if (!mixed_low_pass_channels_.get()) {
      mixed_low_pass_channels_.reset(
          new ChannelBuffer<int16_t>(num_split_frames_, 1));
    }

    DownmixToMono<int16_t, int32_t>(split_channels_const(kBand0To8kHz),
                                    num_split_frames_, num_channels_,
                                    mixed_low_pass_channels_->channels()[0]);
    mixed_low_pass_valid_ = true;
  }
  return mixed_low_pass_channels_->channels()[0];
}

const int16_t* AudioBuffer::low_pass_reference(int channel) const {
  if (!reference_copied_) {
    return NULL;
  }

  return low_pass_reference_channels_->channels()[channel];
}

void AudioBuffer::set_activity(AudioFrame::VADActivity activity) {
  activity_ = activity;
}

AudioFrame::VADActivity AudioBuffer::activity() const {
  return activity_;
}

void AudioBuffer::RestoreNumChannels() {
  mixed_low_pass_valid_ = false;
  reference_copied_ = false;
  activity_ = AudioFrame::kVadUnknown;
  num_channels_ = buffer_num_channels_;
  data_->set_num_channels(buffer_num_channels_);
  if (split_data_.get()) {
    split_data_->set_num_channels(buffer_num_channels_);
  }
}

void AudioBuffer::set_num_channels(size_t num_channels) {
  RTC_DCHECK_GE(buffer_num_channels_, num_channels);
  num_channels_ = num_channels;
  data_->set_num_channels(num_channels);
  if (split_data_.get()) {
    split_data_->set_num_channels(num_channels);
  }
}

// The resampler is only for supporting 48kHz to 16kHz in the reverse stream.
void AudioBuffer::DeinterleaveFrom(AudioFrame* frame) {
  RTC_DCHECK_EQ(frame->num_channels_, input_num_channels_);
  RTC_DCHECK_EQ(frame->samples_per_channel_, input_num_frames_);

  // Initialized lazily because there's a different condition in CopyFrom.
  RestoreNumChannels();
  if ((input_num_frames_ != buffer_num_frames_) && !input_buffer_) {
    input_buffer_.reset(
        new IFChannelBuffer(input_num_frames_, buffer_num_channels_));
  }
  activity_ = frame->vad_activity_;

  int16_t* const* deinterleaved;
  if (input_num_frames_ == buffer_num_frames_) {
    deinterleaved = data_->ibuf()->channels();
  } else {
    deinterleaved = input_buffer_->ibuf()->channels();
  }
  // TODO(yujo): handle muted frames more efficiently.
  if (buffer_num_channels_ == 1) {
    // Downmix and deinterleave simultaneously.
    DownmixInterleavedToMono(frame->data(), input_num_frames_,
                             input_num_channels_, deinterleaved[0]);
  } else {
    RTC_DCHECK_EQ(buffer_num_channels_, input_num_channels_);
    Deinterleave(frame->data(), input_num_frames_, buffer_num_channels_,
                 deinterleaved);
  }

  // Resample.
  if (input_num_frames_ != buffer_num_frames_) {
    for (size_t i = 0; i < buffer_num_channels_; ++i) {
      input_resamplers_[i]->Resample(
          input_buffer_->fbuf_const()->channels()[i], input_num_frames_,
          data_->fbuf()->channels()[i], buffer_num_frames_);
    }
  }
}

void AudioBuffer::InterleaveTo(AudioFrame* frame, bool data_changed) const {
  frame->vad_activity_ = activity_;
  if (!data_changed) {
    return;
  }

  RTC_DCHECK(frame->num_channels_ == num_channels_ || num_channels_ == 1);
  RTC_DCHECK_EQ(frame->samples_per_channel_, output_num_frames_);

  // Resample if necessary.
  IFChannelBuffer* data_ptr = data_.get();
  if (buffer_num_frames_ != output_num_frames_) {
    for (size_t i = 0; i < num_channels_; ++i) {
      output_resamplers_[i]->Resample(
          data_->fbuf()->channels()[i], buffer_num_frames_,
          output_buffer_->fbuf()->channels()[i], output_num_frames_);
    }
    data_ptr = output_buffer_.get();
  }

  // TODO(yujo): handle muted frames more efficiently.
  if (frame->num_channels_ == num_channels_) {
    Interleave(data_ptr->ibuf()->channels(), output_num_frames_, num_channels_,
               frame->mutable_data());
  } else {
    UpmixMonoToInterleaved(data_ptr->ibuf()->channels()[0], output_num_frames_,
                           frame->num_channels_, frame->mutable_data());
  }
}

/* void AudioBuffer::CopyLowPassToReference() {
  reference_copied_ = true;
  if (!low_pass_reference_channels_.get() ||
      low_pass_reference_channels_->num_channels() != num_channels_) {
    low_pass_reference_channels_.reset(
        new ChannelBuffer<int16_t>(num_split_frames_, buffer_num_channels_));
  }
  for (size_t i = 0; i < buffer_num_channels_; i++) {
    memcpy(low_pass_reference_channels_->channels()[i],
           split_bands_const(i)[kBand0To8kHz],
           low_pass_reference_channels_->num_frames_per_band() *
               sizeof(split_bands_const(i)[kBand0To8kHz][0]));
  }
} */

void AudioBuffer::SplitIntoFrequencyBands() {
  splitting_filter_->Analysis(data_.get(), split_data_.get());
}

void AudioBuffer::MergeFrequencyBands() {
  splitting_filter_->Synthesis(split_data_.get(), data_.get());
}

void AudioBuffer::ExportSplitChannelData(
    size_t channel,
    int16_t* const* split_band_data) const {
  for (size_t k = 0; k < num_bands(); ++k) {
    const float* band_data = split_bands_const_f(channel)[k];

    RTC_DCHECK(split_band_data[k]);
    RTC_DCHECK(band_data);
    for (size_t i = 0; i < num_frames_per_band(); ++i) {
      split_band_data[k][i] = FloatS16ToS16(band_data[i]);
    }
  }
}

void AudioBuffer::ImportSplitChannelData(
    size_t channel,
    const int16_t* const* split_band_data) {
  for (size_t k = 0; k < num_bands(); ++k) {
    float* band_data = split_bands_f(channel)[k];
    RTC_DCHECK(split_band_data[k]);
    RTC_DCHECK(band_data);
    for (size_t i = 0; i < num_frames_per_band(); ++i) {
      band_data[i] = split_band_data[k][i];
    }
  }
}

}  // namespace webrtc
