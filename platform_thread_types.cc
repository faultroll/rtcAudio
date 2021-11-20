/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/platform_thread_types.h"
#include "rtc_base/system/compile_magic.h"  // NOLINT(build/include)
#include <stdlib.h>
#include <algorithm> // #include <math.h>
#include "rtc_base/time_utils.h"

#if defined(WEBRTC_LINUX)
#include <sys/prctl.h>
#include <sys/syscall.h>
#endif

namespace rtc {

int ThrdCreate(Thrd *thr, ThrdStartFunction func, void *arg)
{
    return thrd_create(thr, func, arg);
}

Thrd ThrdCurrent(void)
{
    return thrd_current();
}

bool ThrdEqual(Thrd thr0, Thrd thr1)
{
    return thrd_equal(thr0, thr1) == 0;
}

int ThrdJoin(Thrd thr)
{
    return thrd_join(thr, NULL);
}

bool ThrdSleep(int milliseconds)
{
    struct timespec ts;
    TimeToTimespec(&ts, milliseconds);
    return thrd_sleep(&ts, NULL) == 0;
}

void ThrdYield(void)
{
    thrd_sleep(&ts_null, NULL); // thrd_yield();
}

bool ThrdSetPrio(Thrd thr, ThrdPrio prio)
{
#if defined(WEBRTC_WIN)
    int tmp_prio;
    switch (prio) {
        case kLowPrio:
            tmp_prio = THREAD_PRIORITY_BELOW_NORMAL;
            break;
        case kNormalPrio:
            tmp_prio = THREAD_PRIORITY_NORMAL;
            break;
        case kHighPrio:
            tmp_prio = THREAD_PRIORITY_ABOVE_NORMAL;
            break;
        case kHighestPrio:
            tmp_prio = THREAD_PRIORITY_HIGHEST;
            break;
        case kRealtimePrio:
            tmp_prio = THREAD_PRIORITY_TIME_CRITICAL;
            break;
    }
    return SetThreadPrio(thr, tmp_prio) != FALSE;
#else // #elif defined(WEBRTC_POSIX)
    const int policy = SCHED_RR;
    const int min_prio = sched_get_priority_min(policy);
    const int max_prio = sched_get_priority_max(policy);
    if (min_prio == -1 || max_prio == -1)
        return false;

    if (max_prio - min_prio <= 2)
        return false;

    // Convert webrtc priority to system priorities:
    sched_param param;
    const int top_prio = max_prio - 1;
    const int low_prio = min_prio + 1;
    switch (prio) {
        case kLowPrio:
            param.sched_priority = low_prio;
            break;
        case kNormalPrio:
            // The -1 ensures that the kHighPrio is always greater or equal to
            // kNormalPrio.
            param.sched_priority = (low_prio + top_prio - 1) / 2;
            break;
        case kHighPrio:
            param.sched_priority = std::max(top_prio - 2, low_prio);
            break;
        case kHighestPrio:
            param.sched_priority = std::max(top_prio - 1, low_prio);
            break;
        case kRealtimePrio:
            param.sched_priority = top_prio;
            break;
    }
    return pthread_setschedparam(thr, policy, &param) == 0;
#endif  // defined(WEBRTC_WIN)
}

void ThrdSetName(const char *name)
{
#if defined(WEBRTC_WIN)
    struct {
        DWORD dwType;
        LPCSTR szName;
        DWORD dwThreadID;
        DWORD dwFlags;
    } threadname_info = {0x1000, name, static_cast<DWORD>(-1), 0};

    __try {
        RaiseException(0x406D1388, 0, sizeof(threadname_info) / sizeof(DWORD),
                       reinterpret_cast<ULONG_PTR *>(&threadname_info));
    } __except (EXCEPTION_EXECUTE_HANDLER) {  // NOLINT
    }
#else // #elif defined(WEBRTC_POSIX)
    #if defined(WEBRTC_LINUX)
        prctl(PR_SET_NAME, reinterpret_cast<unsigned long>(name));  // NOLINT
    #endif // defined(WEBRTC_LINUX)
    RTC_UNUSED(name);
#endif // defined(WEBRTC_WIN)
}

Tss TssCreate(void)
{
    Tss key;
    tss_create(&key, NULL);
    return key;
}

void TssDelete(Tss key)
{
    tss_delete(key);
}

void *TssGet(Tss key)
{
    return tss_get(key);
}

int TssSet(Tss key, void *val)
{
    return tss_set(key, val);
}

int MtxInit(Mtx *mtx)
{
    return mtx_init(mtx, mtx_plain | mtx_recursive);
}

void MtxDestroy(Mtx *mtx)
{
    mtx_destroy(mtx);
}

int MtxLock(Mtx *mtx)
{
    return mtx_lock(mtx);
}

int MtxTryLock(Mtx *mtx)
{
    return mtx_trylock(mtx);
}

int MtxUnlock(Mtx *mtx)
{
    return mtx_unlock(mtx);
}

}  // namespace rtc
