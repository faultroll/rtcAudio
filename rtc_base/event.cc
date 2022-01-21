/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/event.h"

#if defined(WEBRTC_WIN)
#include <windows.h>
#elif defined(WEBRTC_POSIX)
#include <pthread.h>
#else
#error "Must define either WEBRTC_WIN or WEBRTC_POSIX."
#endif

#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"
#include "rtc_base/optional.h"
// #include "rtc_base/synchronization/yield_policy.h"
// #include "rtc_base/system/warn_current_thread_is_deadlocked.h"

namespace rtc {

#if defined(WEBRTC_WIN)

Event::Event(bool manual_reset, bool initially_signaled) {
  event_handle_ = ::CreateEvent(nullptr,  // Security attributes.
                                manual_reset, initially_signaled,
                                nullptr);  // Name.
  RTC_CHECK(event_handle_);
}

Event::~Event() {
  CloseHandle(event_handle_);
}

void Event::Set() {
  SetEvent(event_handle_);
}

void Event::Reset() {
  ResetEvent(event_handle_);
}

bool Event::Wait(const int give_up_after_ms, int /*warn_after_ms*/) {
  // ScopedYieldPolicy::YieldExecution();
  const DWORD ms = give_up_after_ms == kForever ? INFINITE : give_up_after_ms;
  return (WaitForSingleObject(event_handle_, ms) == WAIT_OBJECT_0);
}

#elif defined(WEBRTC_POSIX)

namespace {

timespec GetTimespec(const int milliseconds_from_now) {
  struct timespec ts, ts_elapsed;
  Timespec(&ts);
  TimeToTimespec(&ts_elapsed, milliseconds_from_now);
  TimespecAfter(&ts, &ts_elapsed);

  return ts;
}

}  // namespace

#if !USE_SEMAPHORE

// On MacOS, clock_gettime is available from version 10.12, and on
// iOS, from version 10.0. So we can't use it yet.
#if defined(WEBRTC_MAC) || defined(WEBRTC_IOS)
#define USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP 0
// On Android, pthread_condattr_setclock is available from version 21. By
// default, we target a new enough version for 64-bit platforms but not for
// 32-bit platforms. For older versions, use
// pthread_cond_timedwait_monotonic_np.
#elif defined(WEBRTC_ANDROID) && (__ANDROID_API__ < 21)
#define USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP 1
#else
#define USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP 0
#endif

Event::Event(bool manual_reset, bool initially_signaled)
    : is_manual_reset_(manual_reset),
      event_status_(initially_signaled) {
  // Cannot use |PTHREAD_MUTEX_RECURSIVE| here
  // |pthread_cond_timedwait| only unlocks one level mutex
  RTC_CHECK(pthread_mutex_init(&event_mutex_, nullptr) == 0);
  pthread_condattr_t cond_attr;
  RTC_CHECK(pthread_condattr_init(&cond_attr) == 0);
#if !USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP
  RTC_CHECK(pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC) == 0);
#endif // USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP
  RTC_CHECK(pthread_cond_init(&event_cond_, &cond_attr) == 0);
  pthread_condattr_destroy(&cond_attr);
}

Event::~Event() {
  pthread_cond_destroy(&event_cond_);
  pthread_mutex_destroy(&event_mutex_);
}

void Event::Set() {
  pthread_mutex_lock(&event_mutex_);
  event_status_ = true;
  pthread_cond_broadcast(&event_cond_);
  pthread_mutex_unlock(&event_mutex_);
}

void Event::Reset() {
  pthread_mutex_lock(&event_mutex_);
  event_status_ = false;
  pthread_mutex_unlock(&event_mutex_);
}

bool Event::Wait(const int give_up_after_ms, const int warn_after_ms) {
  // Instant when we'll log a warning message (because we've been waiting so
  // long it might be a bug), but not yet give up waiting. nullopt if we
  // shouldn't log a warning.
  const rtc::Optional<timespec> warn_ts =
      warn_after_ms == kForever ||
              (give_up_after_ms != kForever && warn_after_ms > give_up_after_ms)
          ? rtc::nullopt
          : rtc::Optional<timespec>(GetTimespec(warn_after_ms));
  // Instant when we'll stop waiting and return an error. nullopt if we should
  // never give up.
  const rtc::Optional<timespec> give_up_ts =
      give_up_after_ms == kForever
          ? rtc::nullopt
          : rtc::Optional<timespec>(GetTimespec(give_up_after_ms));

  // ScopedYieldPolicy::YieldExecution();
  pthread_mutex_lock(&event_mutex_);

  // Wait for `event_cond_` to trigger and `event_status_` to be set, with the
  // given timeout (or without a timeout if none is given).
  const auto wait = [&](const rtc::Optional<timespec> timeout_ts) {
    int error = 0;
    while (!event_status_ && error == 0) {
      if (timeout_ts == rtc::nullopt) {
        error = pthread_cond_wait(&event_cond_, &event_mutex_);
      } else {
#if USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP
        error = pthread_cond_timedwait_monotonic_np(&event_cond_, &event_mutex_, &*timeout_ts);
#else // USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP
        error = pthread_cond_timedwait(&event_cond_, &event_mutex_, &*timeout_ts);
#endif // USE_PTHREAD_COND_TIMEDWAIT_MONOTONIC_NP
      }
    }
    return error;
  };

  int error;
  if (warn_ts == rtc::nullopt) {
    error = wait(give_up_ts);
  } else {
    error = wait(warn_ts);
    if (error == ETIMEDOUT) {
      // webrtc::WarnThatTheCurrentThreadIsProbablyDeadlocked();
      error = wait(give_up_ts);
    }
  }

  // NOTE(liulk): Exactly one thread will auto-reset this event. All
  // the other threads will think it's unsignaled.  This seems to be
  // consistent with auto-reset events in WEBRTC_WIN
  if (error == 0 && !is_manual_reset_)
    event_status_ = false;

  pthread_mutex_unlock(&event_mutex_);

  return (error == 0);
}

#else // USE_SEMAPHORE

Event::Event(bool manual_reset, bool initially_signaled)
    : is_manual_reset_(manual_reset),
      event_status_(initially_signaled) {
  RTC_CHECK(sem_init(&event_sem_, 0, 0) == 0);
}

Event::~Event() {
  sem_destroy(&event_sem_);
}

void Event::Set() {
  // AtomicOps::ReleaseStore(&event_status_, true);
  sem_post(&event_sem_);
}

void Event::Reset() {
  // AtomicOps::ReleaseStore(&event_status_, false);
}

bool Event::Wait(const int give_up_after_ms, int /*warn_after_ms*/) {
  // ScopedYieldPolicy::YieldExecution();
  int error = 0;
  // while (!AtomicOps::AcquireLoad(&event_status_) && error == 0) {
    if (give_up_after_ms == kForever) {
      error = sem_wait(&event_sem_);
    } else {
      timespec timeout_ts = GetTimespec(give_up_after_ms);
      error = sem_timedwait(&event_sem_, &timeout_ts);
    }
  // }
  // if (error == 0 && !is_manual_reset_)
  //   AtomicOps::ReleaseStore(&event_status_, false);
  return (error == 0);
}

#endif // USE_SEMAPHORE

#endif // WEBRTC_WIN

}  // namespace rtc
