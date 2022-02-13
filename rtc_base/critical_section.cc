/*
 *  Copyright 2015 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/critical_section.h"

#include "rtc_base/checks.h"
#include "rtc_base/platform_thread_types.h"

// TODO(tommi): Split this file up to per-platform implementation files.

namespace rtc {

CriticalSection::CriticalSection() {
  MtxInit(&mutex_);

#if RTC_DCHECK_IS_ON
  thread_ = 0;
  recursion_count_ = 0;
#endif  // RTC_DCHECK_IS_ON
}

CriticalSection::~CriticalSection() {
  MtxDestroy(&mutex_);
}

void CriticalSection::Enter() const RTC_EXCLUSIVE_LOCK_FUNCTION() {
  MtxLock(&mutex_);

#if RTC_DCHECK_IS_ON
  if (!recursion_count_) {
    RTC_DCHECK(!thread_);
    thread_ = ThrdCurrent();
  } else {
    RTC_DCHECK(CurrentThreadIsOwner());
  }
  ++recursion_count_;
#endif  // RTC_DCHECK_IS_ON
}

bool CriticalSection::TryEnter() const RTC_EXCLUSIVE_TRYLOCK_FUNCTION(true) {
  if (MtxTryLock(&mutex_) != kThrdSuccess)
    return false;

#if RTC_DCHECK_IS_ON
  if (!recursion_count_) {
    RTC_DCHECK(!thread_);
    thread_ = ThrdCurrent();
  } else {
    RTC_DCHECK(CurrentThreadIsOwner());
  }
  ++recursion_count_;
#endif  // RTC_DCHECK_IS_ON

  return true;
}

void CriticalSection::Leave() const RTC_UNLOCK_FUNCTION() {
  RTC_DCHECK(CurrentThreadIsOwner());

#if RTC_DCHECK_IS_ON
  --recursion_count_;
  RTC_DCHECK(recursion_count_ >= 0);
  if (!recursion_count_)
    thread_ = 0;
#endif  // RTC_DCHECK_IS_ON

  MtxUnlock(&mutex_);
}

bool CriticalSection::CurrentThreadIsOwner() const {
#if RTC_DCHECK_IS_ON
  return ThrdEqual(thread_, ThrdCurrent());
#else
  return true;
#endif  // RTC_DCHECK_IS_ON
}

CritScope::CritScope(const CriticalSection* cs) : cs_(cs) { cs_->Enter(); }
CritScope::~CritScope() { cs_->Leave(); }

TryCritScope::TryCritScope(const CriticalSection* cs)
    : cs_(cs), locked_(cs->TryEnter()) {
#if RTC_DCHECK_IS_ON
  lock_was_called_ = false;
#endif  // RTC_DCHECK_IS_ON
}

TryCritScope::~TryCritScope() {
#if RTC_DCHECK_IS_ON
  RTC_DCHECK(lock_was_called_);
#endif  // RTC_DCHECK_IS_ON
  if (locked_)
    cs_->Leave();
}

bool TryCritScope::locked() const {
#if RTC_DCHECK_IS_ON
  lock_was_called_ = true;
#endif  // RTC_DCHECK_IS_ON
  return locked_;
}

void GlobalLock::Lock() {
  while (AtomicOps::CompareAndSwap(&lock_acquired_, 0, 1)) {
    ThrdYield();
  }
}

void GlobalLock::Unlock() {
  int old_value = AtomicOps::CompareAndSwap(&lock_acquired_, 1, 0);
  RTC_DCHECK_EQ(1, old_value) /* << "Unlock called without calling Lock first" */;
}

GlobalLockScope::GlobalLockScope(GlobalLock* lock)
    : lock_(lock) {
  lock_->Lock();
}

GlobalLockScope::~GlobalLockScope() {
  lock_->Unlock();
}

}  // namespace rtc
