/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/platform_thread.h"

#include "rtc_base/atomic_ops.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"

namespace rtc {
namespace {
#if defined(WEBRTC_WIN)
void CALLBACK RaiseFlag(ULONG_PTR param) {
  *reinterpret_cast<bool*>(param) = true;
}
#endif
}

PlatformThread::PlatformThread(ThreadRunFunction func,
                               void* obj,
                               const char* name /*= "Thread"*/,
                               ThrdPrio priority /*= kNormalPrio*/)
    : run_function_(func), priority_(priority), obj_(obj) {
  RTC_DCHECK(func);
  SetName(name, this);  // default name
  spawned_thread_checker_.Detach();
}

PlatformThread::~PlatformThread() {
  RTC_DCHECK(thread_checker_.IsCurrent());
  RTC_DCHECK(!IsRunning());
}

bool PlatformThread::SetName(const std::string& name, const void* obj) {
  RTC_DCHECK(!IsRunning());
  RTC_DCHECK(!name.empty());
  // TODO(tommi): Consider lowering the limit to 15 (limit on Linux).
  RTC_DCHECK(name.length() < 64);

  name_ = name;
  if (obj) {
    // The %p specifier typically produce at most 16 hex digits, possibly with a
    // 0x prefix. But format is implementation defined, so add some margin.
    char buf[30];
    snprintf(buf, sizeof(buf), " 0x%p", obj);
    name_ += buf;
  }
  return true;
}

int PlatformThread::StartThread(void* param) {
  static_cast<PlatformThread*>(param)->Run();
  return 0;
}

void PlatformThread::Start() {
  RTC_DCHECK(thread_checker_.IsCurrent());
  RTC_DCHECK(!IsRunning()) /* << "Thread already started?" */;

#if defined(WEBRTC_WIN)
  stop_ = false;
#endif

  RTC_CHECK_EQ(kThrdSuccess, ThrdCreate(&thread_, &StartThread, this));

  RTC_CHECK(thread_ != kNullThrd) /* << "CreateThread failed" */;
}

bool PlatformThread::IsRunning() const {
  RTC_DCHECK(thread_checker_.IsCurrent());
  return thread_ != kNullThrd;
}

Thrd PlatformThread::GetThreadRef() const {
  return ThrdCurrent();
}

void PlatformThread::Stop() {
  RTC_DCHECK(thread_checker_.IsCurrent());
  if (!IsRunning())
    return;

  if (!run_function_) {
#if defined(WEBRTC_WIN)
    // Set stop_ to |true| on the worker thread.
    bool queued = QueueAPC(&RaiseFlag, reinterpret_cast<ULONG_PTR>(&stop_));
    // Queuing the APC can fail if the thread is being terminated.
    RTC_CHECK(queued || GetLastError() == ERROR_GEN_FAILURE);
#else
    RTC_CHECK_EQ(1, AtomicOps::Increment(&stop_flag_));
#endif
  }

  RTC_CHECK_EQ(kThrdSuccess, ThrdJoin(thread_));

#if defined(WEBRTC_WIN)
  // Nothing
#else
  if (!run_function_) {
    AtomicOps::ReleaseStore(&stop_flag_, 0);
  }
#endif

  thread_ = kNullThrd;
  spawned_thread_checker_.Detach();
}

// TODO(tommi): Deprecate the loop behavior in PlatformThread.
// * Introduce a new callback type that returns void.
// * Remove potential for a busy loop in PlatformThread.
// * Delegate the responsibility for how to stop the thread, to the
//   implementation that actually uses the thread.
// All implementations will need to be aware of how the thread should be stopped
// and encouraging a busy polling loop, can be costly in terms of power and cpu.
void PlatformThread::Run() {
  // Attach the worker thread checker to this thread.
  RTC_DCHECK(spawned_thread_checker_.IsCurrent());
  ThrdSetName(name_.c_str());

  if (run_function_) {
    // TODO(lgY): find why this happens.
    // avoiding strange bugs running with gdb
    // |thread_| will be 0 even if current thread successfully created
    ThrdSetPrio(ThrdCurrent()/* thread_ */, priority_);
    run_function_(obj_);
    return;
  }

// TODO(tommi): Delete the rest of this function when looping isn't supported.
#if RTC_DCHECK_IS_ON
  // These constants control the busy loop detection algorithm below.
  // |kMaxLoopCount| controls the limit for how many times we allow the loop
  // to run within a period, before DCHECKing.
  // |kPeriodToMeasureMs| controls how long that period is.
  static const int kMaxLoopCount = 1000;
  static const int kPeriodToMeasureMs = 100;
  int64_t loop_stamps[kMaxLoopCount] = {};
  int64_t sequence_nr = 0;
#endif

  do {
    // The interface contract of Start/Stop is that for a successful call to
    // Start, there should be at least one call to the run function.  So we
    // call the function before checking |stop_|.
#if RTC_DCHECK_IS_ON
    auto id = sequence_nr % kMaxLoopCount;
    loop_stamps[id] = TimeMillis();
    if (sequence_nr > kMaxLoopCount) {
      auto compare_id = (id + 1) % kMaxLoopCount;
      auto diff = loop_stamps[id] - loop_stamps[compare_id];
      RTC_DCHECK_GE(diff, 0);
      if (diff < kPeriodToMeasureMs) {
        RTC_NOTREACHED() /* << "This thread is too busy: " << name_ << " " << diff
                         << "ms sequence=" << sequence_nr << " "
                         << loop_stamps[id] << " vs " << loop_stamps[compare_id]
                         << ", " << id << " vs " << compare_id */;
      }
    }
    ++sequence_nr;
#endif

    // Alertable sleep to permit RaiseFlag to run and update |stop_|.
    ThrdYield();

#if defined(WEBRTC_WIN)
  } while (!stop_);
#else
  } while (!AtomicOps::AcquireLoad(&stop_flag_));
#endif
}

bool PlatformThread::SetPriority(ThrdPrio priority) {
#if RTC_DCHECK_IS_ON
  if (run_function_) {
    // The non-deprecated way of how this function gets called, is that it must
    // be called on the worker thread itself.
    RTC_DCHECK(!thread_checker_.IsCurrent());
    RTC_DCHECK(spawned_thread_checker_.IsCurrent());
  } else {
    // In the case of deprecated use of this method, it must be called on the
    // same thread as the PlatformThread object is constructed on.
    RTC_DCHECK(thread_checker_.IsCurrent());
    RTC_DCHECK(IsRunning());
  }
#endif

  return ThrdSetPrio(thread_, priority);
}

#if defined(WEBRTC_WIN)
bool PlatformThread::QueueAPC(PAPCFUNC function, ULONG_PTR data) {
  RTC_DCHECK(thread_checker_.IsCurrent());
  RTC_DCHECK(IsRunning());

  return QueueUserAPC(function, thread_, data) != FALSE;
}
#endif

}  // namespace rtc
