/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC_AGC_MANAGER_DIRECT_H_
#define MODULES_AUDIO_PROCESSING_AGC_AGC_MANAGER_DIRECT_H_

#include <memory>

#include "rtc_base/optional.h"
#include "modules/audio_processing/agc/agc.h"
#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
// #include "rtc_base/gtest_prod_util.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {

class MonoAgc;
// class GainControl;

// Callbacks that need to be injected into AgcManagerDirect to read and control
// the volume values. This is done to remove the VoiceEngine dependency in
// AgcManagerDirect.
// DONE(aluebs): Remove VolumeCallbacks.
/* class VolumeCallbacks {
 public:
  virtual ~VolumeCallbacks() {}
  virtual void SetMicVolume(int volume) = 0;
  virtual int GetMicVolume() = 0;
}; */

// Direct interface to use AGC to set volume and compression values.
// AudioProcessing uses this interface directly to integrate the callback-less
// AGC.
//
// This class is not thread-safe.
class AgcManagerDirect final {
 public:
  // AgcManagerDirect will configure GainControl internally. The user is
  // responsible for processing the audio using it after the call to Process.
  // The operating range of startup_min_level is [12, 255] and any input value
  // outside that range will be clamped.
  AgcManagerDirect(int num_capture_channels,
                   int startup_min_level,
                   int clipped_level_min,
                   bool use_agc2_level_estimation,
                   bool disable_digital_adaptive,
                   int sample_rate_hz);

  ~AgcManagerDirect();

  void Initialize();
  // void SetupDigitalGainControl(GainControl* gain_control) const;

  void AnalyzePreProcess(const AudioBuffer* audio);
  void Process(const AudioBuffer* audio);

  // Call when the capture stream has been muted/unmuted. This causes the
  // manager to disregard all incoming audio; chances are good it's background
  // noise to which we'd like to avoid adapting.
  void SetCaptureMuted(bool muted);
  float voice_probability() const;

  int stream_analog_level() const { return stream_analog_level_; }
  void set_stream_analog_level(int level);
  int num_channels() const { return num_capture_channels_; }
  int sample_rate_hz() const { return sample_rate_hz_; }

  // If available, returns a new compression gain for the digital gain control.
  rtc::Optional<int> GetDigitalComressionGain();

 private:
  /* friend class AgcManagerDirectTest;

  FRIEND_TEST_ALL_PREFIXES(AgcManagerDirectStandaloneTest,
                           DisableDigitalDisablesDigital);
  FRIEND_TEST_ALL_PREFIXES(AgcManagerDirectStandaloneTest,
                           AgcMinMicLevelExperiment);

  // Dependency injection for testing. Don't delete |agc| as the memory is owned
  // by the manager.
  AgcManagerDirect(Agc* agc,
                   int startup_min_level,
                   int clipped_level_min,
                   int sample_rate_hz); */

  void AnalyzePreProcess(const float* const* audio, size_t samples_per_channel);

  void AggregateChannelLevels();

  std::unique_ptr<ApmDataDumper> data_dumper_;
  static int instance_counter_;
  const bool use_min_channel_level_;
  const int sample_rate_hz_;
  const int num_capture_channels_;
  const bool disable_digital_adaptive_;

  int frames_since_clipped_;
  int stream_analog_level_ = 0;
  bool capture_muted_;
  int channel_controlling_gain_ = 0;

  std::vector<std::unique_ptr<MonoAgc>> channel_agcs_;
  std::vector<rtc::Optional<int>> new_compressions_to_set_;

  RTC_DISALLOW_COPY_AND_ASSIGN(AgcManagerDirect);
};

class MonoAgc {
 public:
  MonoAgc(ApmDataDumper* data_dumper,
          int startup_min_level,
          int clipped_level_min,
          bool use_agc2_level_estimation,
          bool disable_digital_adaptive,
          int min_mic_level);
  ~MonoAgc();

  void Initialize();
  void SetCaptureMuted(bool muted);

  void HandleClipping();

  void Process(const int16_t* audio,
               size_t samples_per_channel,
               int sample_rate_hz);

  void set_stream_analog_level(int level) { stream_analog_level_ = level; }
  int stream_analog_level() const { return stream_analog_level_; }
  float voice_probability() const { return agc_->voice_probability(); }
  void ActivateLogging() { log_to_histograms_ = true; }
  rtc::Optional<int> new_compression() const {
    return new_compression_to_set_;
  }

  // Only used for testing.
  void set_agc(Agc* agc) { agc_.reset(agc); }
  int min_mic_level() const { return min_mic_level_; }
  int startup_min_level() const { return startup_min_level_; }

 private:
  // Sets a new microphone level, after first checking that it hasn't been
  // updated by the user, in which case no action is taken.
  void SetLevel(int new_level);

  // Set the maximum level the AGC is allowed to apply. Also updates the
  // maximum compression gain to compensate. The level must be at least
  // |kClippedLevelMin|.
  void SetMaxLevel(int level);

  int CheckVolumeAndReset();
  void UpdateGain();
  void UpdateCompressor();

  const int min_mic_level_;
  const bool disable_digital_adaptive_;
  std::unique_ptr<Agc> agc_;
  int level_ = 0;
  int max_level_;
  int max_compression_gain_;
  int target_compression_;
  int compression_;
  float compression_accumulator_;
  bool capture_muted_ = false;
  bool check_volume_on_next_process_ = true;
  bool startup_ = true;
  int startup_min_level_;
  int calls_since_last_gain_log_ = 0;
  int stream_analog_level_ = 0;
  rtc::Optional<int> new_compression_to_set_;
  bool log_to_histograms_ = false;
  const int clipped_level_min_;

  RTC_DISALLOW_COPY_AND_ASSIGN(MonoAgc);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC_AGC_MANAGER_DIRECT_H_
