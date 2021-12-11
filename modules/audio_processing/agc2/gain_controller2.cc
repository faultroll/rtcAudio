/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_processing/agc2/gain_controller2.h"

#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/include/audio_frame_view.h"
#include "modules/audio_processing/include/common.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/atomic_ops.h"
#include "rtc_base/checks.h"

namespace webrtc {

int GainController2::instance_count_ = 0;

GainController2::GainController2()
    : data_dumper_(
          new ApmDataDumper(rtc::AtomicOps::Increment(&instance_count_))),
      gain_applier_(/*hard_clip_samples=*/false,
                    /*initial_gain_factor=*/0.f),
      limiter_(static_cast<size_t>(48000), data_dumper_.get(), "Agc2") {
  if (config_.adaptive_digital.enabled) {
    adaptive_agc_.reset(new AdaptiveAgc(data_dumper_.get()));
  }
}

GainController2::~GainController2() {}

void GainController2::Initialize(int sample_rate_hz) {
  RTC_DCHECK(sample_rate_hz == AudioProcessing::kSampleRate8kHz ||
             sample_rate_hz == AudioProcessing::kSampleRate16kHz ||
             sample_rate_hz == AudioProcessing::kSampleRate32kHz ||
             sample_rate_hz == AudioProcessing::kSampleRate48kHz);
  limiter_.SetSampleRate(sample_rate_hz);
  data_dumper_->InitiateNewSetOfRecordings();
  data_dumper_->DumpRaw("sample_rate_hz", sample_rate_hz);
}

void GainController2::Process(AudioBuffer* audio) {
  AudioFrameView<float> float_frame(audio->channels_f(), audio->num_channels(),
                                    audio->num_frames());
  // Apply fixed gain first, then the adaptive one.
  gain_applier_.ApplyGain(float_frame);
  if (adaptive_agc_) {
    adaptive_agc_->Process(float_frame, limiter_.LastAudioLevel());
  }
  limiter_.Process(float_frame);
}

void GainController2::NotifyAnalogLevel(int level) {
  if (analog_level_ != level && adaptive_agc_) {
    adaptive_agc_->Reset();
  }
  analog_level_ = level;
}

void GainController2::ApplyConfig(const Agc2Config& config) {
  RTC_DCHECK(Agc2Config::Validate(config)) /* << " the invalid config was " << ToString(config) */;
  config_ = config;
  if (config.fixed_digital.gain_db != config_.fixed_digital.gain_db) {
    // Reset the limiter to quickly react on abrupt level changes caused by
    // large changes of the fixed gain.
    limiter_.Reset();
  }
  gain_applier_.SetGainFactor(DbToRatio(config_.fixed_digital.gain_db));
  if (config_.adaptive_digital.enabled) {
    adaptive_agc_.reset(new AdaptiveAgc(data_dumper_.get(), config_));
  } else {
    adaptive_agc_.reset();
  }
}

}  // namespace webrtc
