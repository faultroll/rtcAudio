/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_PLATFORM_THREAD_H_
#define RTC_BASE_PLATFORM_THREAD_H_

#include <string>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/synchronization/sequence_checker.h"

namespace rtc {

// Callback function that the spawned thread will enter once spawned.
// A return value of false is interpreted as that the function has no
// more work to do and that the thread can be released.
typedef void (*ThreadRunFunction)(void*);

// Represents a simple worker thread.  The implementation must be assumed
// to be single threaded, meaning that all methods of the class, must be
// called from the same thread, including instantiation.
class PlatformThread {
 public:
  PlatformThread(ThreadRunFunction func,
                 void* obj,
                 const char* name = "Thread",
                 ThrdPrio priority = kNormalPrio);
  virtual ~PlatformThread();

  // Sets the thread's name. Must be called before Start().
  // If |obj| is non-null, its value is appended to |name|.
  const std::string& name() const { return name_; }
  bool SetName(const std::string& name, const void* obj);

  // Spawns a thread and tries to set thread priority according to the priority
  // from when CreateThread was called.
  void Start();

  bool IsRunning() const;

  // Returns an identifier for the worker thread that can be used to do
  // thread checks.
  Thrd GetThreadRef() const;

  // Stops (joins) the spawned thread.
  void Stop();
  
  // Set the priority of the thread. Must be called when thread is running.
  // TODO(tommi): Make private and only allow public support via ctor.
  bool SetPriority(ThrdPrio priority);

 protected:
#if defined(WEBRTC_WIN)
  // Exposed to derived classes to allow for special cases specific to Windows.
  bool QueueAPC(PAPCFUNC apc_function, ULONG_PTR data);
#endif

 private:
  void Run();

  ThreadRunFunction const run_function_ = nullptr;
  const ThrdPrio priority_ = kNormalPrio;
  void* const obj_;
  // TODO(pbos): Make sure call sites use string literals and update to a const
  // char* instead of a std::string.
  /* const */ std::string name_;
  rtc::SequenceChecker thread_checker_;
  rtc::SequenceChecker spawned_thread_checker_;
  
  static int StartThread(void* param);
  Thrd thread_ = kNullThrd;
#if defined(WEBRTC_WIN)
  bool stop_ = false;
#else
  // An atomic flag that we use to stop the thread. Only modified on the
  // controlling thread and checked on the worker thread.
  volatile int stop_flag_ = 0;
#endif  // defined(WEBRTC_WIN)
  RTC_DISALLOW_COPY_AND_ASSIGN(PlatformThread);
};

}  // namespace rtc

#endif  // RTC_BASE_PLATFORM_THREAD_H_
