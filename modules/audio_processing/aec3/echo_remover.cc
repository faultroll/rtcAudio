/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "modules/audio_processing/aec3/echo_remover.h"

#include <math.h>
#include <stddef.h>

#include <algorithm>
// #include <array>
#include <cmath>
#include <memory>

#include "rtc_base/view.h"
#include "modules/audio_processing/aec3/aec3_common.h"
#include "modules/audio_processing/aec3/aec3_fft.h"
#include "modules/audio_processing/aec3/aec_state.h"
#include "modules/audio_processing/aec3/comfort_noise_generator.h"
#include "modules/audio_processing/aec3/echo_path_variability.h"
#include "modules/audio_processing/aec3/echo_remover_metrics.h"
#include "modules/audio_processing/aec3/fft_data.h"
#include "modules/audio_processing/aec3/render_buffer.h"
#include "modules/audio_processing/aec3/render_signal_analyzer.h"
#include "modules/audio_processing/aec3/residual_echo_estimator.h"
#include "modules/audio_processing/aec3/subtractor.h"
#include "modules/audio_processing/aec3/subtractor_output.h"
#include "modules/audio_processing/aec3/suppression_filter.h"
#include "modules/audio_processing/aec3/suppression_gain.h"
#include "modules/audio_processing/logging/apm_data_dumper.h"
#include "rtc_base/atomic_ops.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"
// #include "rtc_base/logging.h"

namespace webrtc {

namespace {

// Maximum number of channels for which the capture channel data is stored on
// the stack. If the number of channels are larger than this, they are stored
// using scratch memory that is pre-allocated on the heap. The reason for this
// partitioning is not to waste heap space for handling the more common numbers
// of channels, while at the same time not limiting the support for higher
// numbers of channels by enforcing the capture channel data to be stored on the
// stack using a fixed maximum value.
constexpr size_t kMaxNumChannelsOnStack = 2;

// Chooses the number of channels to store on the heap when that is required due
// to the number of capture channels being larger than the pre-defined number
// of channels to store on the stack.
size_t NumChannelsOnHeap(size_t num_capture_channels) {
  return num_capture_channels > kMaxNumChannelsOnStack ? num_capture_channels : 0;
}

void LinearEchoPower(const FftData& E,
                     const FftData& Y,
                     RTC_VIEW(float) /* kFftLengthBy2Plus1 */ S2) {
  for (size_t k = 0; k < E.re_view.size(); ++k) {
    S2[k] = (Y.re_view[k] - E.re_view[k]) * (Y.re_view[k] - E.re_view[k]) +
            (Y.im_view[k] - E.im_view[k]) * (Y.im_view[k] - E.im_view[k]);
  }
}

// Fades between two input signals using a fix-sized transition.
void SignalTransition(RTC_VIEW(const float) from,
                      RTC_VIEW(const float) to,
                      RTC_VIEW(float) out) {
  if (from.data() == to.data() && from.size() == to.size()) {
    RTC_DCHECK_EQ(to.size(), out.size());
    std::copy(to.begin(), to.end(), out.begin());
  } else {
    constexpr size_t kTransitionSize = 30;
    constexpr float kOneByTransitionSizePlusOne = 1.f / (kTransitionSize + 1);

    RTC_DCHECK_EQ(from.size(), to.size());
    RTC_DCHECK_EQ(from.size(), out.size());
    RTC_DCHECK_LE(kTransitionSize, out.size());

    for (size_t k = 0; k < kTransitionSize; ++k) {
      float a = (k + 1) * kOneByTransitionSizePlusOne;
      out[k] = a * to[k] + (1.f - a) * from[k];
    }

    std::copy(to.begin() + kTransitionSize, to.end(),
              out.begin() + kTransitionSize);
  }
}

// Computes a windowed (square root Hanning) padded FFT and updates the related
// memory.
void WindowedPaddedFft(const Aec3Fft& fft,
                       RTC_VIEW(const float) v,
                       RTC_VIEW(float) v_old,
                       FftData* V) {
  fft.PaddedFft(v, v_old, Aec3Fft::Window::kSqrtHanning, V);
  std::copy(v.begin(), v.end(), v_old.begin());
}

// Class for removing the echo from the capture signal.
class EchoRemoverImpl final : public EchoRemover {
 public:
  EchoRemoverImpl(const EchoCanceller3Config& config,
                  int sample_rate_hz,
                  size_t num_render_channels,
                  size_t num_capture_channels);
  ~EchoRemoverImpl() override;

  void GetMetrics(EchoControl::Metrics* metrics) const override;

  // Removes the echo from a block of samples from the capture signal. The
  // supplied render signal is assumed to be pre-aligned with the capture
  // signal.
  void ProcessCapture(
      EchoPathVariability echo_path_variability,
      bool capture_signal_saturation,
      const rtc::Optional<DelayEstimate>& external_delay,
      RenderBuffer* render_buffer,
      std::vector<std::vector<std::vector<float>>>* linear_output,
      std::vector<std::vector<std::vector<float>>>* capture) override;

  // Updates the status on whether echo leakage is detected in the output of the
  // echo remover.
  void UpdateEchoLeakageStatus(bool leakage_detected) override {
    echo_leakage_detected_ = leakage_detected;
  }

 private:
  // Selects which of the coarse and refined linear filter outputs that is most
  // appropriate to pass to the suppressor and forms the linear filter output by
  // smoothly transition between those.
  void FormLinearFilterOutput(const SubtractorOutput& subtractor_output,
                              RTC_VIEW(float) output);

  static int instance_count_;
  const EchoCanceller3Config config_;
  const Aec3Fft fft_;
  std::unique_ptr<ApmDataDumper> data_dumper_;
  const Aec3Optimization optimization_;
  const int sample_rate_hz_;
  const size_t num_render_channels_;
  const size_t num_capture_channels_;
  const bool use_coarse_filter_output_;
  Subtractor subtractor_;
  SuppressionGain suppression_gain_;
  ComfortNoiseGenerator cng_;
  SuppressionFilter suppression_filter_;
  RenderSignalAnalyzer render_signal_analyzer_;
  ResidualEchoEstimator residual_echo_estimator_;
  bool echo_leakage_detected_ = false;
  AecState aec_state_;
  EchoRemoverMetrics metrics_;
  std::vector<std::array<float, kFftLengthBy2>> e_old_;
  std::vector<std::array<float, kFftLengthBy2>> y_old_;
  size_t block_counter_ = 0;
  int gain_change_hangover_ = 0;
  bool refined_filter_output_last_selected_ = true;

  std::vector<std::array<float, kFftLengthBy2>> e_heap_;
  std::vector<std::array<float, kFftLengthBy2Plus1>> Y2_heap_;
  std::vector<std::array<float, kFftLengthBy2Plus1>> E2_heap_;
  std::vector<std::array<float, kFftLengthBy2Plus1>> R2_heap_;
  std::vector<std::array<float, kFftLengthBy2Plus1>> S2_linear_heap_;
  std::vector<FftData> Y_heap_;
  std::vector<FftData> E_heap_;
  std::vector<FftData> comfort_noise_heap_;
  std::vector<FftData> high_band_comfort_noise_heap_;
  std::vector<SubtractorOutput> subtractor_output_heap_;

  RTC_DISALLOW_COPY_AND_ASSIGN(EchoRemoverImpl);
};

int EchoRemoverImpl::instance_count_ = 0;

EchoRemoverImpl::EchoRemoverImpl(const EchoCanceller3Config& config,
                                 int sample_rate_hz,
                                 size_t num_render_channels,
                                 size_t num_capture_channels)
    : config_(config),
      fft_(),
      data_dumper_(
          new ApmDataDumper(rtc::AtomicOps::Increment(&instance_count_))),
      optimization_(DetectOptimization()),
      sample_rate_hz_(sample_rate_hz),
      num_render_channels_(num_render_channels),
      num_capture_channels_(num_capture_channels),
      use_coarse_filter_output_(
          config_.filter.enable_coarse_filter_output_usage),
      subtractor_(config,
                  num_render_channels_,
                  num_capture_channels_,
                  data_dumper_.get(),
                  optimization_),
      suppression_gain_(config_,
                        optimization_,
                        sample_rate_hz,
                        num_capture_channels),
      cng_(config_, optimization_, num_capture_channels_),
      suppression_filter_(optimization_,
                          sample_rate_hz_,
                          num_capture_channels_),
      render_signal_analyzer_(config_),
      residual_echo_estimator_(config_, num_render_channels),
      aec_state_(config_, num_capture_channels_),
      e_old_(num_capture_channels_, {0.f}),
      y_old_(num_capture_channels_, {0.f}),
      e_heap_(NumChannelsOnHeap(num_capture_channels_), {0.f}),
      Y2_heap_(NumChannelsOnHeap(num_capture_channels_)),
      E2_heap_(NumChannelsOnHeap(num_capture_channels_)),
      R2_heap_(NumChannelsOnHeap(num_capture_channels_)),
      S2_linear_heap_(NumChannelsOnHeap(num_capture_channels_)),
      Y_heap_(NumChannelsOnHeap(num_capture_channels_)),
      E_heap_(NumChannelsOnHeap(num_capture_channels_)),
      comfort_noise_heap_(NumChannelsOnHeap(num_capture_channels_)),
      high_band_comfort_noise_heap_(NumChannelsOnHeap(num_capture_channels_)),
      subtractor_output_heap_(NumChannelsOnHeap(num_capture_channels_)) {
  RTC_DCHECK(ValidFullBandRate(sample_rate_hz));
}

EchoRemoverImpl::~EchoRemoverImpl() {}

void EchoRemoverImpl::GetMetrics(EchoControl::Metrics* metrics) const {
  // Echo return loss (ERL) is inverted to go from gain to attenuation.
  metrics->echo_return_loss = -10.0 * std::log10(aec_state_.ErlTimeDomain());
  metrics->echo_return_loss_enhancement =
      Log2TodB(aec_state_.FullBandErleLog2());
}

void EchoRemoverImpl::ProcessCapture(
    EchoPathVariability echo_path_variability,
    bool capture_signal_saturation,
    const rtc::Optional<DelayEstimate>& external_delay,
    RenderBuffer* render_buffer,
    std::vector<std::vector<std::vector<float>>>* linear_output,
    std::vector<std::vector<std::vector<float>>>* capture) {
  ++block_counter_;
  const std::vector<std::vector<std::vector<float>>>& x = render_buffer->Block(0);
  std::vector<std::vector<std::vector<float>>>* y = capture;
  RTC_DCHECK(render_buffer);
  RTC_DCHECK(y);
  RTC_DCHECK_EQ(x.size(), NumBandsForRate(sample_rate_hz_));
  RTC_DCHECK_EQ(y->size(), NumBandsForRate(sample_rate_hz_));
  RTC_DCHECK_EQ(x[0].size(), num_render_channels_);
  RTC_DCHECK_EQ((*y)[0].size(), num_capture_channels_);
  RTC_DCHECK_EQ(x[0][0].size(), kBlockSize);
  RTC_DCHECK_EQ((*y)[0][0].size(), kBlockSize);

  // RTC_VIEW(RTC_VIEW(float)/* [kFftLengthBy2] */) e;
  /* RTC_VIEW(float[kFftLengthBy2]) e;
  RTC_VIEW(float[kFftLengthBy2Plus1]) Y2;
  RTC_VIEW(float[kFftLengthBy2Plus1]) E2;
  RTC_VIEW(float[kFftLengthBy2Plus1]) R2;
  RTC_VIEW(float[kFftLengthBy2Plus1]) S2_linear; */
  RTC_VIEW(FftData) Y;
  RTC_VIEW(FftData) E;
  RTC_VIEW(FftData) comfort_noise;
  RTC_VIEW(FftData) high_band_comfort_noise;
  RTC_VIEW(SubtractorOutput) subtractor_output;
  // Stack allocated data to use when the number of channels is low.
  /* float e_stack[kMaxNumChannelsOnStack][kFftLengthBy2];
  float Y2_stack[kMaxNumChannelsOnStack][kFftLengthBy2Plus1];
  float E2_stack[kMaxNumChannelsOnStack][kFftLengthBy2Plus1];
  float R2_stack[kMaxNumChannelsOnStack][kFftLengthBy2Plus1];
  float S2_linear_stack[kMaxNumChannelsOnStack][kFftLengthBy2Plus1]; */
  std::vector<std::array<float, kFftLengthBy2>> e_stack(kMaxNumChannelsOnStack);
  std::vector<std::array<float, kFftLengthBy2Plus1>> Y2_stack(kMaxNumChannelsOnStack);
  std::vector<std::array<float, kFftLengthBy2Plus1>> E2_stack(kMaxNumChannelsOnStack);
  std::vector<std::array<float, kFftLengthBy2Plus1>> R2_stack(kMaxNumChannelsOnStack);
  std::vector<std::array<float, kFftLengthBy2Plus1>> S2_linear_stack(kMaxNumChannelsOnStack);
  FftData Y_stack[kMaxNumChannelsOnStack];
  FftData E_stack[kMaxNumChannelsOnStack];
  FftData comfort_noise_stack[kMaxNumChannelsOnStack];
  FftData high_band_comfort_noise_stack[kMaxNumChannelsOnStack];
  SubtractorOutput subtractor_output_stack[kMaxNumChannelsOnStack];
  // error: 'e' declared as reference but not initializea
  std::vector<std::array<float, kFftLengthBy2>>& e = e_stack;
  std::vector<std::array<float, kFftLengthBy2Plus1>>& Y2 = Y2_stack;
  std::vector<std::array<float, kFftLengthBy2Plus1>>& E2 = E2_stack;
  std::vector<std::array<float, kFftLengthBy2Plus1>>& R2 = R2_stack;
  std::vector<std::array<float, kFftLengthBy2Plus1>>& S2_linear = S2_linear_stack;
  if (NumChannelsOnHeap(num_capture_channels_) == 0) {
    /* e = RTC_MAKE_VIEW(float[kFftLengthBy2])(
        e_stack, num_capture_channels_);
    Y2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        Y2_stack, num_capture_channels_);
    E2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        E2_stack, num_capture_channels_);
    R2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        R2_stack, num_capture_channels_);
    S2_linear = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        S2_linear_stack, num_capture_channels_); */
    /* e = e_stack;
    Y2 = Y2_stack;
    E2 = E2_stack;
    R2 = R2_stack;
    S2_linear = S2_linear_stack; */
    Y = RTC_MAKE_VIEW(FftData)(
        Y_stack, num_capture_channels_);
    E = RTC_MAKE_VIEW(FftData)(
        E_stack, num_capture_channels_);
    comfort_noise = RTC_MAKE_VIEW(FftData)(
        comfort_noise_stack, num_capture_channels_);
    high_band_comfort_noise = RTC_MAKE_VIEW(FftData)(
        high_band_comfort_noise_stack, num_capture_channels_);
    subtractor_output = RTC_MAKE_VIEW(SubtractorOutput)(
        subtractor_output_stack, num_capture_channels_);
  } else {
    // If the stack-allocated space is too small, use the heap for storing the
    // microphone data.
    // error: cannot convert 'std::array<float, 64ul>*' to 'float (*)[64]' in initialization
    // can be done like below, but we don't need View of each array in two dimension array
    /* std::vector<RTC_VIEW(float)> e_ch(num_capture_channels_);
    for (size_t ch = 0; ch < num_capture_channels_; ++ch) {
        e_ch[ch] = RTC_MAKE_VIEW(float)(e_heap_.data()[ch]);
    }
    e = RTC_MAKE_VIEW(RTC_VIEW(float))(
        e_ch.data(), num_capture_channels_); */
    /* e = RTC_MAKE_VIEW(float[kFftLengthBy2])(
        e_heap_.data(), num_capture_channels_);
    Y2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        Y2_heap_.data(), num_capture_channels_);
    E2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        E2_heap_.data(), num_capture_channels_);
    R2 = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        R2_heap_.data(), num_capture_channels_);
    S2_linear = RTC_MAKE_VIEW(float[kFftLengthBy2Plus1])(
        S2_linear_heap_.data(), num_capture_channels_); */
    e = e_heap_;
    Y2 = Y2_heap_;
    E2 = E2_heap_;
    R2 = R2_heap_;
    S2_linear = S2_linear_heap_;
    Y = RTC_MAKE_VIEW(FftData)(
        Y_heap_.data(), num_capture_channels_);
    E = RTC_MAKE_VIEW(FftData)(
        E_heap_.data(), num_capture_channels_);
    comfort_noise = RTC_MAKE_VIEW(FftData)(
        comfort_noise_heap_.data(), num_capture_channels_);
    high_band_comfort_noise = RTC_MAKE_VIEW(FftData)(
        high_band_comfort_noise_heap_.data(), num_capture_channels_);
    subtractor_output = RTC_MAKE_VIEW(SubtractorOutput)(
        subtractor_output_heap_.data(), num_capture_channels_);
  }

  data_dumper_->DumpWav(
      "aec3_echo_remover_capture_input", kBlockSize, &(*y)[0][0][0], 16000, 1);
  data_dumper_->DumpWav(
      "aec3_echo_remover_render_input", kBlockSize, &x[0][0][0], 16000, 1);
  data_dumper_->DumpRaw(
      "aec3_echo_remover_capture_input", RTC_MAKE_VIEW(const float)((*y)[0][0]));
  data_dumper_->DumpRaw(
      "aec3_echo_remover_render_input", RTC_MAKE_VIEW(const float)(x[0][0]));

  aec_state_.UpdateCaptureSaturation(capture_signal_saturation);

  if (echo_path_variability.AudioPathChanged()) {
    // Ensure that the gain change is only acted on once per frame.
    if (echo_path_variability.gain_change) {
      if (gain_change_hangover_ == 0) {
        constexpr int kMaxBlocksPerFrame = 3;
        gain_change_hangover_ = kMaxBlocksPerFrame;
        /* rtc::LoggingSeverity log_level =
            config_.delay.log_warning_on_delay_changes ? rtc::LS_WARNING
                                                       : rtc::LS_VERBOSE;
        RTC_LOG_V(log_level)
            << "Gain change detected at block " << block_counter_; */
      } else {
        echo_path_variability.gain_change = false;
      }
    }

    subtractor_.HandleEchoPathChange(echo_path_variability);
    aec_state_.HandleEchoPathChange(echo_path_variability);

    if (echo_path_variability.delay_change !=
        EchoPathVariability::DelayAdjustment::kNone) {
      suppression_gain_.SetInitialState(true);
    }
  }
  if (gain_change_hangover_ > 0) {
    --gain_change_hangover_;
  }

  // Analyze the render signal.
  render_signal_analyzer_.Update(*render_buffer,
                                 aec_state_.MinDirectPathFilterDelay());

  // State transition.
  if (aec_state_.TransitionTriggered()) {
    subtractor_.ExitInitialState();
    suppression_gain_.SetInitialState(false);
  }

  // Perform linear echo cancellation.
  subtractor_.Process(*render_buffer, (*y)[0], render_signal_analyzer_,
                      aec_state_, subtractor_output);

  // Compute spectra.
  for (size_t ch = 0; ch < num_capture_channels_; ++ch) {
    FormLinearFilterOutput(subtractor_output[ch], e[ch]);
    WindowedPaddedFft(fft_, (*y)[0][ch], y_old_[ch], &Y[ch]);
    WindowedPaddedFft(fft_, e[ch], e_old_[ch], &E[ch]);
    LinearEchoPower(E[ch], Y[ch], RTC_MAKE_VIEW(float)(S2_linear[ch]));
    Y[ch].Spectrum(optimization_, Y2[ch]);
    E[ch].Spectrum(optimization_, E2[ch]);
  }

  // Optionally return the linear filter output.
  if (linear_output) {
    RTC_DCHECK_GE(1, linear_output->size());
    RTC_DCHECK_EQ(num_capture_channels_, linear_output[0].size());
    for (size_t ch = 0; ch < num_capture_channels_; ++ch) {
      RTC_DCHECK_EQ(kBlockSize, (*linear_output)[0][ch].size());
      std::copy(e[ch].begin(), e[ch].end(), (*linear_output)[0][ch].begin());
    }
  }

  // Update the AEC state information.
  aec_state_.Update(external_delay, subtractor_.FilterFrequencyResponses(),
                    subtractor_.FilterImpulseResponses(), *render_buffer, E2,
                    Y2, subtractor_output);

  // Choose the linear output.
  const auto& Y_fft = aec_state_.UseLinearFilterOutput() ? E : Y;

  data_dumper_->DumpWav(
      "aec3_output_linear", kBlockSize, &(*y)[0][0][0], 16000, 1);
  data_dumper_->DumpWav(
      "aec3_output_linear2", kBlockSize, &e[0][0], 16000, 1);

  // Estimate the residual echo power.
  residual_echo_estimator_.Estimate(aec_state_, *render_buffer, S2_linear, Y2, R2);

  // Estimate the comfort noise.
  cng_.Compute(aec_state_.SaturatedCapture(), Y2, comfort_noise, high_band_comfort_noise);

  // Suppressor nearend estimate.
  if (aec_state_.UsableLinearEstimate()) {
    // E2 is bound by Y2.
    for (size_t ch = 0; ch < num_capture_channels_; ++ch) {
      std::transform(E2[ch].begin(), E2[ch].end(), Y2[ch].begin(),
                     E2[ch].begin(),
                     [](float a, float b) { return std::min(a, b); });
    }
  }
  const auto& nearend_spectrum = aec_state_.UsableLinearEstimate() ? E2 : Y2;

  // Suppressor echo estimate.
  const auto& echo_spectrum =
      aec_state_.UsableLinearEstimate() ? S2_linear : R2;

  // Compute preferred gains.
  float high_bands_gain;
  float G[kFftLengthBy2Plus1];
  RTC_VIEW(float) G_view = RTC_MAKE_VIEW(float)(G);
  suppression_gain_.GetGain(nearend_spectrum, echo_spectrum, R2,
                            cng_.NoiseSpectrum(), render_signal_analyzer_,
                            aec_state_, x, &high_bands_gain, G_view);

  suppression_filter_.ApplyGain(comfort_noise, high_band_comfort_noise, G_view,
                                high_bands_gain, Y_fft, y);

  // Update the metrics.
  metrics_.Update(aec_state_, cng_.NoiseSpectrum()[0], G_view);

  // Debug outputs for the purpose of development and analysis.
  data_dumper_->DumpWav(
      "aec3_echo_estimate", kBlockSize, &subtractor_output[0].s_refined[0], 16000, 1);
  data_dumper_->DumpRaw(
      "aec3_output", RTC_MAKE_VIEW(const float)((*y)[0][0]));
  data_dumper_->DumpRaw(
      "aec3_narrow_render", render_signal_analyzer_.NarrowPeakBand() ? 1 : 0);
  data_dumper_->DumpRaw(
      "aec3_N2", RTC_MAKE_VIEW(const float)(cng_.NoiseSpectrum()[0]));
  data_dumper_->DumpRaw(
      "aec3_suppressor_gain", RTC_MAKE_VIEW(const float)(G_view));
  data_dumper_->DumpWav(
      "aec3_output", RTC_MAKE_VIEW(const float)(&(*y)[0][0][0], kBlockSize), 16000, 1);
  data_dumper_->DumpRaw(
      "aec3_using_subtractor_output[0]", aec_state_.UseLinearFilterOutput() ? 1 : 0);
  data_dumper_->DumpRaw(
      "aec3_E2", RTC_MAKE_VIEW(const float)(E2[0]));
  data_dumper_->DumpRaw(
      "aec3_S2_linear", RTC_MAKE_VIEW(const float)(S2_linear[0]));
  data_dumper_->DumpRaw(
      "aec3_Y2", RTC_MAKE_VIEW(const float)(Y2[0]));
  data_dumper_->DumpRaw(
      "aec3_X2", RTC_MAKE_VIEW(const float)(
          render_buffer->Spectrum(aec_state_.MinDirectPathFilterDelay())[/*channel=*/0]));
  data_dumper_->DumpRaw(
      "aec3_R2", RTC_MAKE_VIEW(const float)(R2[0]));
  data_dumper_->DumpRaw(
      "aec3_filter_delay", aec_state_.MinDirectPathFilterDelay());
  data_dumper_->DumpRaw(
      "aec3_capture_saturation", aec_state_.SaturatedCapture() ? 1 : 0);
}

void EchoRemoverImpl::FormLinearFilterOutput(
    const SubtractorOutput& subtractor_output,
    RTC_VIEW(float) output) {
  RTC_DCHECK_EQ(subtractor_output.e_refined.size(), output.size());
  RTC_DCHECK_EQ(subtractor_output.e_coarse.size(), output.size());
  bool use_refined_output = true;
  if (use_coarse_filter_output_) {
    // As the output of the refined adaptive filter generally should be better
    // than the coarse filter output, add a margin and threshold for when
    // choosing the coarse filter output.
    if (subtractor_output.e2_coarse < 0.9f * subtractor_output.e2_refined &&
        subtractor_output.y2 > 30.f * 30.f * kBlockSize &&
        (subtractor_output.s2_refined > 60.f * 60.f * kBlockSize ||
         subtractor_output.s2_coarse > 60.f * 60.f * kBlockSize)) {
      use_refined_output = false;
    } else {
      // If the refined filter is diverged, choose the filter output that has
      // the lowest power.
      if (subtractor_output.e2_coarse < subtractor_output.e2_refined &&
          subtractor_output.y2 < subtractor_output.e2_refined) {
        use_refined_output = false;
      }
    }
  }

  SignalTransition(refined_filter_output_last_selected_
                       ? subtractor_output.e_refined
                       : subtractor_output.e_coarse,
                   use_refined_output ? subtractor_output.e_refined
                                      : subtractor_output.e_coarse,
                   output);
  refined_filter_output_last_selected_ = use_refined_output;
}

}  // namespace

EchoRemover* EchoRemover::Create(const EchoCanceller3Config& config,
                                 int sample_rate_hz,
                                 size_t num_render_channels,
                                 size_t num_capture_channels) {
  return new EchoRemoverImpl(config, sample_rate_hz, num_render_channels,
                             num_capture_channels);
}

}  // namespace webrtc
