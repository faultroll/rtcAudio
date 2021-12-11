/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_PLATFORM_THREAD_TYPES_H_
#define RTC_BASE_PLATFORM_THREAD_TYPES_H_

#include "rtc_base/features/thread_c.h"

namespace rtc {

// Function return values
enum {
    kThrdSuccess    = thrd_success,
    kThrdNomem      = thrd_nomem,
    kThrdTimedout   = thrd_timedout,
    kThrdBusy       = thrd_busy,
    kThrdError      = thrd_error,
};

// Thread
typedef thrd_t Thrd;
static const Thrd kNullThrd = thrd_null;
// Any thread that is started with the |ThrdCreate| function must be
// started through a function of this type.
typedef int (*ThrdStartFunction)(void *arg);
// Create a new thread
int ThrdCreate(Thrd *thr, ThrdStartFunction func, void *arg);
// Retrieves a reference to the current thread. On Windows, this is the same
// as CurrentThreadId. On other platforms it's the pthread_t returned by
// pthread_self().
Thrd ThrdCurrent(void);
// Compares two thread identifiers for equality.
bool ThrdEqual(Thrd thr0, Thrd thr1);
// Wait for a thread to terminate
int ThrdJoin(Thrd thr);
// Sleeps the calling thread for the specified number of milliseconds, during
// which time no processing is performed. Returns false if sleeping was
// interrupted by a signal (POSIX only).
bool ThrdSleep(int milliseconds);
// Yield execution to another thread. Permit other threads to run,
// even if current thread would ordinarily continue to run.
void ThrdYield(void);
typedef enum ThrdPrio_ {
    kLowPrio = thrd_priority_low,
    kNormalPrio = thrd_priority_normal,
    kHighPrio = thrd_priority_high,
    kHighestPrio = thrd_priority_highest,
    kRealtimePrio = thrd_priority_realtime,
} ThrdPrio;
// Set the priority of the thread. Must be called when thread is running.
bool ThrdSetPrio(Thrd thr, ThrdPrio prio);
// Sets the current thread name.
void ThrdSetName(const char *name);

// TLs(Thread Local-storage) or TsS(Thread-specific Storage)
typedef tss_t Tss;
static const Tss kNullTss = tss_null;
// Create a TsS
Tss TssCreate(void);
// Delete a TsS
void TssDelete(Tss key);
// Get the value for a TsS
void *TssGet(Tss key);
// Set the value for a TsS
int TssSet(Tss key, void *val);

// Mutex
typedef mtx_t Mtx;
// static const Mtx kNullMtx = mtx_null;
// Create a mutex object
int MtxInit(Mtx *mtx);
// Release any resources used by the given mutex
void MtxDestroy(Mtx *mtx);
// Lock the given mutex
// Blocks until the given mutex can be locked. If the mutex is non-recursive, and
// the calling thread already has a lock on the mutex, this call will block forever.
int MtxLock(Mtx *mtx);
// Try to lock the given mutex
// The specified mutex shall support either test and return or timeout. If the
// mutex is already locked, the function returns without blocking.
int MtxTryLock(Mtx *mtx);
// Unlock the given mutex
int MtxUnlock(Mtx *mtx);

}  // namespace rtc

#endif  // RTC_BASE_PLATFORM_THREAD_TYPES_H_
