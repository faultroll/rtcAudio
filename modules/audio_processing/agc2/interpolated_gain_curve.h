/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_
#define MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_

// #include <array>
#include <string>

#include "rtc_base/view.h"
#include "modules/audio_processing/agc2/agc2_common.h"
#include "rtc_base/constructor_magic.h"
// #include "rtc_base/gtest_prod_util.h"
#include "system_wrappers/include/metrics.h"

namespace webrtc {

class ApmDataDumper;

constexpr float kInputLevelScalingFactor = 32768.0f;

// Defined as DbfsToLinear(kLimiterMaxInputLevelDbFs)
constexpr float kMaxInputLevelLinear = static_cast<float>(36766.300710566735);

// Interpolated gain curve using under-approximation to avoid saturation.
//
// The goal of this class is allowing fast look ups to get an accurate
// estimates of the gain to apply given an estimated input level.
class InterpolatedGainCurve {
 public:
  enum class GainCurveRegion {
    kIdentity = 0,
    kKnee = 1,
    kLimiter = 2,
    kSaturation = 3
  };

  struct Stats {
    // Region in which the output level equals the input one.
    size_t look_ups_identity_region = 0;
    // Smoothing between the identity and the limiter regions.
    size_t look_ups_knee_region = 0;
    // Limiter region in which the output and input levels are linearly related.
    size_t look_ups_limiter_region = 0;
    // Region in which saturation may occur since the input level is beyond the
    // maximum expected by the limiter.
    size_t look_ups_saturation_region = 0;
    // True if stats have been populated.
    bool available = false;

    // The current region, and for how many frames the level has been
    // in that region.
    GainCurveRegion region = GainCurveRegion::kIdentity;
    int64_t region_duration_frames = 0;
  };

  InterpolatedGainCurve(ApmDataDumper* apm_data_dumper,
                        std::string histogram_name_prefix);
  ~InterpolatedGainCurve();

  Stats get_stats() const { return stats_; }

  // Given a non-negative input level (linear scale), a scalar factor to apply
  // to a sub-frame is returned.
  // Levels above kLimiterMaxInputLevelDbFs will be reduced to 0 dBFS
  // after applying this gain
  float LookUpGainToApply(float input_level) const;

 private:
  // For comparing 'approximation_params_*_' with ones computed by
  // ComputeInterpolatedGainCurve.
  /* FRIEND_TEST_ALL_PREFIXES(AutomaticGainController2InterpolatedGainCurve,
                           CheckApproximationParams); */

  struct RegionLogger {
    metrics::Histogram* identity_histogram;
    metrics::Histogram* knee_histogram;
    metrics::Histogram* limiter_histogram;
    metrics::Histogram* saturation_histogram;

    RegionLogger(std::string identity_histogram_name,
                 std::string knee_histogram_name,
                 std::string limiter_histogram_name,
                 std::string saturation_histogram_name);

    ~RegionLogger();

    void LogRegionStats(const InterpolatedGainCurve::Stats& stats) const;
  } region_logger_;

  void UpdateStats(float input_level) const;

  ApmDataDumper* const apm_data_dumper_;

  static const float approximation_params_x_[kInterpolatedGainCurveTotalPoints];
  RTC_VIEW(const float) approximation_params_x_view_ = 
    RTC_MAKE_VIEW(const float)(approximation_params_x_);
  static const float approximation_params_m_[kInterpolatedGainCurveTotalPoints];
  RTC_VIEW(const float) approximation_params_m_view_ = 
    RTC_MAKE_VIEW(const float)(approximation_params_m_);
  static const float approximation_params_q_[kInterpolatedGainCurveTotalPoints];
  RTC_VIEW(const float) approximation_params_q_view_ = 
    RTC_MAKE_VIEW(const float)(approximation_params_q_);

  // Stats.
  mutable Stats stats_;

  RTC_DISALLOW_COPY_AND_ASSIGN(InterpolatedGainCurve);
};

}  // namespace webrtc

#endif  // MODULES_AUDIO_PROCESSING_AGC2_INTERPOLATED_GAIN_CURVE_H_
