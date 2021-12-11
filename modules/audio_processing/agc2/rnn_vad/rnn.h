/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RNN_H_
#define MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RNN_H_

#include <stddef.h>
#include <stdint.h>
// #include <sys/types.h>

// #include <array>
#include <vector>

#include "rtc_base/view.h"
#include "modules/audio_processing/agc2/rnn_vad/common.h"
// #include "rtc_base/system/arch.h"
#include "rtc_base/constructor_magic.h"

namespace webrtc {
namespace rnn_vad {

// Maximum number of units for a fully-connected layer. This value is used to
// over-allocate space for fully-connected layers output vectors (implemented as
// std::array). The value should equal the number of units of the largest
// fully-connected layer.
constexpr size_t kFullyConnectedLayersMaxUnits = 24;

// Maximum number of units for a recurrent layer. This value is used to
// over-allocate space for recurrent layers state vectors (implemented as
// std::array). The value should equal the number of units of the largest
// recurrent layer.
constexpr size_t kRecurrentLayersMaxUnits = 24;

// Fully-connected layer.
class FullyConnectedLayer {
 public:
  FullyConnectedLayer(size_t input_size,
                      size_t output_size,
                      RTC_VIEW(const int8_t) bias,
                      RTC_VIEW(const int8_t) weights,
                      float (*const activation_function)(float),
                      Optimization optimization);
  ~FullyConnectedLayer();
  size_t input_size() const { return input_size_; }
  size_t output_size() const { return output_size_; }
  Optimization optimization() const { return optimization_; }
  RTC_VIEW(const float) GetOutput() const;
  // Computes the fully-connected layer output.
  void ComputeOutput(RTC_VIEW(const float) input);

 private:
  const size_t input_size_;
  const size_t output_size_;
  const std::vector<float> bias_;
  const std::vector<float> weights_;
  float (*const activation_function_)(float);
  // The output vector of a recurrent layer has length equal to |output_size_|.
  // However, for efficiency, over-allocation is used.
  float output_[kFullyConnectedLayersMaxUnits];
  RTC_VIEW(float) output_view_;
  const Optimization optimization_;

  RTC_DISALLOW_COPY_AND_ASSIGN(FullyConnectedLayer);
};

// Recurrent layer with gated recurrent units (GRUs) with sigmoid and ReLU as
// activation functions for the update/reset and output gates respectively.
class GatedRecurrentLayer {
 public:
  GatedRecurrentLayer(size_t input_size,
                      size_t output_size,
                      RTC_VIEW(const int8_t) bias,
                      RTC_VIEW(const int8_t) weights,
                      RTC_VIEW(const int8_t) recurrent_weights,
                      Optimization optimization);
  ~GatedRecurrentLayer();
  size_t input_size() const { return input_size_; }
  size_t output_size() const { return output_size_; }
  Optimization optimization() const { return optimization_; }
  RTC_VIEW(const float) GetOutput() const;
  void Reset();
  // Computes the recurrent layer output and updates the status.
  void ComputeOutput(RTC_VIEW(const float) input);

 private:
  const size_t input_size_;
  const size_t output_size_;
  const std::vector<float> bias_;
  const std::vector<float> weights_;
  const std::vector<float> recurrent_weights_;
  // The state vector of a recurrent layer has length equal to |output_size_|.
  // However, to avoid dynamic allocation, over-allocation is used.
  float state_[kRecurrentLayersMaxUnits];
  RTC_VIEW(float) state_view_;
  const Optimization optimization_;

  RTC_DISALLOW_COPY_AND_ASSIGN(GatedRecurrentLayer);
};

// Recurrent network based VAD.
class RnnBasedVad {
 public:
  RnnBasedVad();
  ~RnnBasedVad();
  void Reset();
  // Compute and returns the probability of voice (range: [0.0, 1.0]).
  float ComputeVadProbability(
      RTC_VIEW(const float) /* kFeatureVectorSize */ feature_vector,
      bool is_silence);

 private:
  FullyConnectedLayer input_layer_;
  GatedRecurrentLayer hidden_layer_;
  FullyConnectedLayer output_layer_;

  RTC_DISALLOW_COPY_AND_ASSIGN(RnnBasedVad);
};

}  // namespace rnn_vad
}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_RNN_VAD_RNN_H_
