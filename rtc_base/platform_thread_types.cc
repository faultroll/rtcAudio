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
    return thrd_set_priority(thr, prio) == thrd_success;
}

void ThrdSetName(const char *name)
{
    thrd_set_name(name);
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
