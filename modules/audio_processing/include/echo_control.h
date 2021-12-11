/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef API_AUDIO_ECHO_CONTROL_H_
#define API_AUDIO_ECHO_CONTROL_H_

#include <memory>

namespace webrtc {

// Interface for an acoustic echo cancellation (AEC) submodule.
class AudioBuffer;
class EchoControl {
 public:
  // Analysis (not changing) of the render signal.
  virtual void AnalyzeRender(AudioBuffer* render) = 0;

  // Analysis (not changing) of the capture signal.
  virtual void AnalyzeCapture(AudioBuffer* capture) = 0;

  // Processes the capture signal in order to remove the echo.
  virtual void ProcessCapture(AudioBuffer* capture, bool level_change) = 0;

  // As above, but also returns the linear filter output.
  virtual void ProcessCapture(AudioBuffer* capture,
                              AudioBuffer* linear_output,
                              bool level_change) = 0;

  struct Metrics {
    double echo_return_loss;
    double echo_return_loss_enhancement;
    int delay_ms;
  };

  // Collect current metrics from the echo controller.
  virtual Metrics GetMetrics() const = 0;

  // Provides an optional external estimate of the audio buffer delay.
  virtual void SetAudioBufferDelay(int delay_ms) = 0;

  // Returns wheter the signal is altered.
  virtual bool ActiveProcessing() const = 0;

  virtual ~EchoControl() {}
};

// Interface for a factory that creates EchoControllers.
class EchoControlFactory {
 public:
  virtual std::unique_ptr<EchoControl> Create(int sample_rate_hz,
                                              int num_render_channels,
                                              int num_capture_channels) = 0;
  virtual ~EchoControlFactory() {}
};

// The acoustic echo cancellation (AEC) component provides better performance
// than AECM but also requires more processing power and is dependent on delay
// stability and reporting accuracy. As such it is well-suited and recommended
// for PC and IP phone applications.
//
// Not recommended to be enabled on the server-side.
// struct AecCore;
class EchoCancellation {
 public:
  // EchoCancellation and EchoControlMobile may not be enabled simultaneously.
  // Enabling one will disable the other.
  virtual int Enable(bool enable) = 0;
  virtual bool is_enabled() const = 0;

  // Differences in clock speed on the primary and reverse streams can impact
  // the AEC performance. On the client-side, this could be seen when different
  // render and capture devices are used, particularly with webcams.
  //
  // This enables a compensation mechanism, and requires that
  // set_stream_drift_samples() be called.
  virtual int enable_drift_compensation(bool enable) = 0;
  virtual bool is_drift_compensation_enabled() const = 0;

  // Sets the difference between the number of samples rendered and captured by
  // the audio devices since the last call to |ProcessStream()|. Must be called
  // if drift compensation is enabled, prior to |ProcessStream()|.
  virtual void set_stream_drift_samples(int drift) = 0;
  virtual int stream_drift_samples() const = 0;

  enum SuppressionLevel {
    kLowSuppression,
    kModerateSuppression,
    kHighSuppression
  };

  // Sets the aggressiveness of the suppressor. A higher level trades off
  // double-talk performance for increased echo suppression.
  virtual int set_suppression_level(SuppressionLevel level) = 0;
  virtual SuppressionLevel suppression_level() const = 0;

  // Returns false if the current frame almost certainly contains no echo
  // and true if it _might_ contain echo.
  virtual bool stream_has_echo() const = 0;

  // Enables the computation of various echo metrics. These are obtained
  // through |GetMetrics()|.
  virtual int enable_metrics(bool enable) = 0;
  virtual bool are_metrics_enabled() const = 0;

  // TODO(ivoc): Remove when the calling code no longer uses the old Statistics
  //             API.
  struct Statistic {
    int instant = 0;  // Instantaneous value.
    int average = 0;  // Long-term average.
    int maximum = 0;  // Long-term maximum.
    int minimum = 0;  // Long-term minimum.
  };

  // Each statistic is reported in dB.
  // P_far:  Far-end (render) signal power.
  // P_echo: Near-end (capture) echo signal power.
  // P_out:  Signal power at the output of the AEC.
  // P_a:    Internal signal power at the point before the AEC's non-linear
  //         processor.
  struct Metrics {
    // RERL = ERL + ERLE
    Statistic residual_echo_return_loss;

    // ERL = 10log_10(P_far / P_echo)
    Statistic echo_return_loss;

    // ERLE = 10log_10(P_echo / P_out)
    Statistic echo_return_loss_enhancement;

    // (Pre non-linear processing suppression) A_NLP = 10log_10(P_echo / P_a)
    Statistic a_nlp;

    // Fraction of time that the AEC linear filter is divergent, in a 1-second
    // non-overlapped aggregation window.
    float divergent_filter_fraction;
  };

  // Deprecated. Use GetStatistics on the AudioProcessing interface instead.
  // TODO(ajm): discuss the metrics update period.
  virtual int GetMetrics(Metrics* metrics) = 0;

  // Enables computation and logging of delay values. Statistics are obtained
  // through |GetDelayMetrics()|.
  virtual int enable_delay_logging(bool enable) = 0;
  virtual bool is_delay_logging_enabled() const = 0;

  // The delay metrics consists of the delay |median| and the delay standard
  // deviation |std|. It also consists of the fraction of delay estimates
  // |fraction_poor_delays| that can make the echo cancellation perform poorly.
  // The values are aggregated until the first call to |GetDelayMetrics()| and
  // afterwards aggregated and updated every second.
  // Note that if there are several clients pulling metrics from
  // |GetDelayMetrics()| during a session the first call from any of them will
  // change to one second aggregation window for all.
  // Deprecated. Use GetStatistics on the AudioProcessing interface instead.
  virtual int GetDelayMetrics(int* median, int* std) = 0;
  // Deprecated. Use GetStatistics on the AudioProcessing interface instead.
  virtual int GetDelayMetrics(int* median,
                              int* std,
                              float* fraction_poor_delays) = 0;

  // Returns a pointer to the low level AEC component.  In case of multiple
  // channels, the pointer to the first one is returned.  A NULL pointer is
  // returned when the AEC component is disabled or has not been initialized
  // successfully.
  // virtual struct AecCore* aec_core() const = 0;

 protected:
  virtual ~EchoCancellation() {}
};

// The acoustic echo control for mobile (AECM) component is a low complexity
// robust option intended for use on mobile devices.
//
// Not recommended to be enabled on the server-side.
class EchoControlMobile {
 public:
  // EchoCancellation and EchoControlMobile may not be enabled simultaneously.
  // Enabling one will disable the other.
  virtual int Enable(bool enable) = 0;
  virtual bool is_enabled() const = 0;

  // Recommended settings for particular audio routes. In general, the louder
  // the echo is expected to be, the higher this value should be set. The
  // preferred setting may vary from device to device.
  enum RoutingMode {
    kQuietEarpieceOrHeadset,
    kEarpiece,
    kLoudEarpiece,
    kSpeakerphone,
    kLoudSpeakerphone
  };

  // Sets echo control appropriate for the audio routing |mode| on the device.
  // It can and should be updated during a call if the audio routing changes.
  virtual int set_routing_mode(RoutingMode mode) = 0;
  virtual RoutingMode routing_mode() const = 0;

  // Comfort noise replaces suppressed background noise to maintain a
  // consistent signal level.
  virtual int enable_comfort_noise(bool enable) = 0;
  virtual bool is_comfort_noise_enabled() const = 0;

  // A typical use case is to initialize the component with an echo path from a
  // previous call. The echo path is retrieved using |GetEchoPath()|, typically
  // at the end of a call. The data can then be stored for later use as an
  // initializer before the next call, using |SetEchoPath()|.
  //
  // Controlling the echo path this way requires the data |size_bytes| to match
  // the internal echo path size. This size can be acquired using
  // |echo_path_size_bytes()|. |SetEchoPath()| causes an entire reset, worth
  // noting if it is to be called during an ongoing call.
  //
  // It is possible that version incompatibilities may result in a stored echo
  // path of the incorrect size. In this case, the stored path should be
  // discarded.
  virtual int SetEchoPath(const void* echo_path, size_t size_bytes) = 0;
  virtual int GetEchoPath(void* echo_path, size_t size_bytes) const = 0;

  // The returned path size is guaranteed not to change for the lifetime of
  // the application.
  static size_t echo_path_size_bytes();

 protected:
  virtual ~EchoControlMobile() {}
};

}  // namespace webrtc

#endif  // API_AUDIO_ECHO_CONTROL_H_
