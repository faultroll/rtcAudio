/*
 *  Copyright 2005 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_TIME_UTILS_H_
#define RTC_BASE_TIME_UTILS_H_

#include "rtc_base/features/time_c.h"

namespace rtc {

static const int64_t kNumMillisecsPerSec = MS_PER_SEC;
static const int64_t kNumMicrosecsPerSec = US_PER_SEC;
static const int64_t kNumNanosecsPerSec  = NS_PER_SEC;
static const int64_t kNumMicrosecsPerMillisec = US_PER_MS;
static const int64_t kNumNanosecsPerMillisec  = NS_PER_MS;
static const int64_t kNumNanosecsPerMicrosec  = NS_PER_US;

// System time is |CLOCK_MONOTONIC|
// Returns the actual system time, even if a clock is set for testing.
// Useful for timeouts while using a test clock, or for logging.
/* int64_t SystemTimeNanos(); */
int64_t SystemTimeMillis();

// UTC time is |CLOCK_REALTIME|
// Return the number of microseconds since January 1, 1970, UTC.
// Useful mainly when producing logs to be correlated with other
// devices, and when the devices in question all have properly
// synchronized clocks.
//
// Note that this function obeys the system's idea about what the time
// is. It is not guaranteed to be monotonic; it will jump in case the
// system time is changed, e.g., by some other process calling
// settimeofday. Always use TimeMillis(), not this function, for
// measuring time intervals and timeouts.
/* int64_t UTCTimeNanos(); */
int64_t UTCTimeMillis();

// Returns the current time in milliseconds in 64 bits.
int64_t TimeMillis();
// Deprecated. Do not use this in any new code.
/* inline int64_t Time() {
  return TimeMillis();
} */

// Returns the current time in microseconds.
int64_t TimeMicros();

// Returns the current time in nanoseconds.
int64_t TimeNanos();


// Returns a future timestamp, 'elapsed' milliseconds from now.
int64_t TimeAfter(int64_t elapsed);

// Number of milliseconds that would elapse between 'earlier' and 'later'
// timestamps.  The value is negative if 'later' occurs before 'earlier'.
int64_t TimeDiff(int64_t later, int64_t earlier);

// The number of milliseconds that have elapsed since 'earlier'.
inline int64_t TimeSince(int64_t earlier) {
  return TimeMillis() - earlier;
}

// The number of milliseconds that will elapse between now and 'later'.
inline int64_t TimeUntil(int64_t later) {
  return later - TimeMillis();
}

/* class TimestampWrapAroundHandler {
 public:
  TimestampWrapAroundHandler();

  int64_t Unwrap(uint32_t ts);

 private:
  uint32_t last_ts_;
  int64_t num_wrap_;
};

// Interval of time from the range [min, max] inclusive.
class IntervalRange {
 public:
  IntervalRange() : min_(0), max_(0) {}
  IntervalRange(int min, int max) : min_(min), max_(max) {
    RTC_DCHECK_LE(min, max);
  }

  int min() const { return min_; }
  int max() const { return max_; }

  std::string ToString() const {
    std::stringstream ss;
    ss << "[" << min_ << "," << max_ << "]";
    return ss.str();
  }

  bool operator==(const IntervalRange& o) const {
    return min_ == o.min_ && max_ == o.max_;
  }

 private:
  int min_;
  int max_;
}; */

// tm calc
// Convert from struct tm, which is relative to 1900-01-01 00:00 to number of
// seconds from 1970-01-01 00:00 ("epoch").  Don't return time_t since that
// is still 32 bits on many systems.
int64_t TmToTime(const struct tm *tm);

// Returns the current timespec
void Timespec(struct timespec *ts);

// Normalize timespec
void TimespecNormalize(struct timespec *ts);

// Returns a future timestamp, |elapsed| milliseconds from now.
void TimespecAfter(struct timespec *ts, struct timespec *elapsed);

// Convert milliseconds to timespec
void TimeToTimespec(struct timespec *ts, int64_t milliseconds);

// Convert timespec to milliseconds
int64_t TimespecToTime(struct timespec *ts);

}  // namespace rtc

#endif  // RTC_BASE_TIME_UTILS_H_
