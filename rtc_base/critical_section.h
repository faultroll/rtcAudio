/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_CRITICAL_SECTION_H_
#define RTC_BASE_CRITICAL_SECTION_H_

#include "rtc_base/atomic_ops.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/system/compile_magic.h"  // NOLINT(build/include)

namespace rtc {

// Locking methods (Enter, TryEnter, Leave)are const to permit protecting
// members inside a const context without requiring mutable CriticalSections
// everywhere. CriticalSection is reentrant lock.
class RTC_LOCKABLE RTC_EXPORT CriticalSection {
 public:
  CriticalSection();
  ~CriticalSection();

  void Enter() const RTC_EXCLUSIVE_LOCK_FUNCTION();
  bool TryEnter() const RTC_EXCLUSIVE_TRYLOCK_FUNCTION(true);
  void Leave() const RTC_UNLOCK_FUNCTION();

 private:
  // Use only for RTC_DCHECKing.
  bool CurrentThreadIsOwner() const;

  mutable Mtx mutex_;
#if RTC_DCHECK_IS_ON
  mutable Thrd thread_;           // Only used by RTC_DCHECKs.
  mutable int recursion_count_;   // Only used by RTC_DCHECKs.
#endif  // RTC_DCHECK_IS_ON
};

// CritScope, for serializing execution through a scope.
class RTC_SCOPED_LOCKABLE CritScope {
 public:
  explicit CritScope(const CriticalSection* cs) RTC_EXCLUSIVE_LOCK_FUNCTION(cs);
  ~CritScope() RTC_UNLOCK_FUNCTION();

 private:
  const CriticalSection* const cs_;
  RTC_DISALLOW_COPY_AND_ASSIGN(CritScope);
};

// Tries to lock a critical section on construction via
// CriticalSection::TryEnter, and unlocks on destruction if the
// lock was taken. Never blocks.
//
// IMPORTANT: Unlike CritScope, the lock may not be owned by this thread in
// subsequent code. Users *must* check locked() to determine if the
// lock was taken. If you're not calling locked(), you're doing it wrong!
class TryCritScope {
 public:
  explicit TryCritScope(const CriticalSection* cs);
  ~TryCritScope();
  RTC_CHECKRETURN bool locked() const;

 private:
  const CriticalSection* const cs_;
  const bool locked_;
#if RTC_DCHECK_IS_ON
  mutable bool lock_was_called_;  // Only used by RTC_DCHECK
#endif  // RTC_DCHECK_IS_ON.
  RTC_DISALLOW_COPY_AND_ASSIGN(TryCritScope);
};

// A POD lock used to protect global variables. Do NOT use for other purposes.
// No custom constructor or private data member should be added.
class RTC_LOCKABLE GlobalLock {
 public:
  constexpr GlobalLock() 
      : lock_acquired_(0) {}

  void Lock() RTC_EXCLUSIVE_LOCK_FUNCTION();
  void Unlock() RTC_UNLOCK_FUNCTION();

 private:
  volatile int lock_acquired_;
};

// GlobalLockScope, for serializing execution through a scope.
class RTC_SCOPED_LOCKABLE GlobalLockScope {
 public:
  explicit GlobalLockScope(GlobalLock* lock) RTC_EXCLUSIVE_LOCK_FUNCTION(lock);
  ~GlobalLockScope() RTC_UNLOCK_FUNCTION();

 private:
  GlobalLock* const lock_;
  RTC_DISALLOW_COPY_AND_ASSIGN(GlobalLockScope);
};

}  // namespace rtc

#endif  // RTC_BASE_CRITICAL_SECTION_H_
