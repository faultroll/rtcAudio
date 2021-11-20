/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/memory/aligned_malloc.h"

#include "rtc_base/features/align_c.h"

namespace webrtc {

void* GetRightAlign(const void* pointer, size_t alignment) {
  if (!pointer) {
    return NULL;
  }
  if (!ALIGN_VALID(alignment)) {
    return NULL;
  }
  uintptr_t start_pos = reinterpret_cast<uintptr_t>(pointer);
  return reinterpret_cast<void*>(ALIGN_UP(start_pos, alignment));
}

void* AlignedMalloc(size_t size, size_t alignment) {
  return aligned_alloc(size, alignment);
}

void AlignedFree(void* mem_block) {
  aligned_free(mem_block);
}

}  // namespace webrtc
