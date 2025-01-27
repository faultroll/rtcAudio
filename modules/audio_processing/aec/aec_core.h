/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

/*
 * Specifies the interface for the AEC core.
 */

#ifndef MODULES_AUDIO_PROCESSING_AEC_AEC_CORE_H_
#define MODULES_AUDIO_PROCESSING_AEC_AEC_CORE_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
// namespace webrtc {
extern "C" {
#endif

#define FRAME_LEN 80
#define PART_LEN 64               // Length of partition
#define PART_LEN1 (PART_LEN + 1)  // Unique fft coefficients
#define PART_LEN2 (PART_LEN * 2)  // Length of partition * 2
#define NUM_HIGH_BANDS_MAX 2      // Max number of high bands

typedef float complex_t[2];
// For performance reasons, some arrays of complex numbers are replaced by twice
// as long arrays of float, all the real parts followed by all the imaginary
// ones (complex_t[SIZE] -> float[2][SIZE]). This allows SIMD optimizations and
// is better than two arrays (one for the real parts and one for the imaginary
// parts) as this other way would require two pointers instead of one and cause
// extra register spilling. This also allows the offsets to be calculated at
// compile time.

// Metrics
enum { kOffsetLevel = -100 };

typedef struct Stats {
  float instant;
  float average;
  float min;
  float max;
  float sum;
  float hisum;
  float himean;
  size_t counter;
  size_t hicounter;
} Stats;

typedef struct AecCore AecCore; // handle

AecCore* WebRtcAec_CreateAec(int instance_count);  // Returns NULL on error.
void WebRtcAec_FreeAec(AecCore* aec);
int WebRtcAec_InitAec(AecCore* aec, int sampFreq);

void WebRtcAec_BufferFarendBlock(AecCore* aec, const float* farend);
void WebRtcAec_ProcessFrames(AecCore* aec,
                             const float* const* nearend,
                             size_t num_bands,
                             size_t num_samples,
                             int knownDelay,
                             float* const* out);

// A helper function to call adjust the farend buffer size.
// Returns the number of elements the size was decreased with, and adjusts
// |system_delay| by the corresponding amount in ms.
int WebRtcAec_AdjustFarendBufferSizeAndSystemDelay(AecCore* aec,
                                                   int size_decrease);

// Calculates the median, standard deviation and amount of poor values among the
// delay estimates aggregated up to the first call to the function. After that
// first call the metrics are aggregated and updated every second. With poor
// values we mean values that most likely will cause the AEC to perform poorly.
// TODO(bjornv): Consider changing tests and tools to handle constant
// constant aggregation window throughout the session instead.
int WebRtcAec_GetDelayMetricsCore(AecCore* self,
                                  int* median,
                                  int* std,
                                  float* fraction_poor_delays);

// Returns the echo state (1: echo, 0: no echo).
int WebRtcAec_echo_state(AecCore* self);

// Gets statistics of the echo metrics ERL, ERLE, A_NLP.
void WebRtcAec_GetEchoStats(AecCore* self,
                            Stats* erl,
                            Stats* erle,
                            Stats* a_nlp,
                            float* divergent_filter_fraction);

// Sets local configuration modes.
void WebRtcAec_SetConfigCore(AecCore* self,
                             int nlp_mode,
                             int metrics_mode,
                             int delay_logging);

// Non-zero enables, zero disables.
void WebRtcAec_enable_delay_agnostic(AecCore* self, int enable);

// Returns non-zero if delay agnostic (i.e., signal based delay estimation) is
// enabled and zero if disabled.
int WebRtcAec_delay_agnostic_enabled(AecCore* self);

// Turns on/off the refined adaptive filter feature.
void WebRtcAec_enable_refined_adaptive_filter(AecCore* self, bool enable);

// Returns whether the refined adaptive filter is enabled.
bool WebRtcAec_refined_adaptive_filter_enabled(const AecCore* self);

// Enables or disables extended filter mode. Non-zero enables, zero disables.
void WebRtcAec_enable_extended_filter(AecCore* self, int enable);

// Returns non-zero if extended filter mode is enabled and zero if disabled.
int WebRtcAec_extended_filter_enabled(AecCore* self);

// Returns the current |system_delay|, i.e., the buffered difference between
// far-end and near-end.
int WebRtcAec_system_delay(AecCore* self);

// Sets the |system_delay| to |value|.  Note that if the value is changed
// improperly, there can be a performance regression.  So it should be used with
// care.
void WebRtcAec_SetSystemDelay(AecCore* self, int delay);

#ifdef __cplusplus
} // extern "C"
// } // namespace webrtc
#endif

#endif  // MODULES_AUDIO_PROCESSING_AEC_AEC_CORE_H_
