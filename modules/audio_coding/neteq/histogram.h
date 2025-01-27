/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_CODING_NETEQ_HISTOGRAM_H_
#define MODULES_AUDIO_CODING_NETEQ_HISTOGRAM_H_

#include <stddef.h>  // size_t
#include <stdbool.h>
#include <stdint.h>

#include <vector>

namespace webrtc {
namespace neteq {

class Histogram {
 public:
  // Creates histogram with capacity |num_buckets| and |forget_factor| in Q15.
  Histogram(size_t num_buckets, int forget_factor);

  virtual ~Histogram();

  // Resets the histogram to the default start distribution.
  virtual void Reset();

  // Add entry in bucket |index|.
  virtual void Add(int index);

  // Calculates the quantile at |probability| (in Q30) of the histogram
  // distribution.
  virtual int Quantile(int probability);

  // Apply compression or stretching to the histogram.
  virtual void Scale(int old_bucket_width, int new_bucket_width);

  // Returns the number of buckets in the histogram.
  virtual int NumBuckets() const;

  // Returns the probability for each bucket in Q30.
  std::vector<int> buckets() const { return buckets_; }

  int forget_factor() const { return base_forget_factor_; }

  // Made public for testing.
  static std::vector<int> ScaleBuckets(const std::vector<int>& buckets,
                                       int old_bucket_width,
                                       int new_bucket_width);

 private:
  std::vector<int> buckets_;
  int forget_factor_;  // Q15
  const int base_forget_factor_;
};

}  // namespace neteq
}  // namespace webrtc

#endif  // MODULES_AUDIO_CODING_NETEQ_HISTOGRAM_H_
