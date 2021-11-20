/*
 *  Copyright 2006 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// C version of checks

#include "rtc_base/checks.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(WEBRTC_WIN)
#include <windows.h>
#define LAST_SYSTEM_ERROR (::GetLastError())
#elif defined(WEBRTC_POSIX)
#include <errno.h>
#define LAST_SYSTEM_ERROR (errno)
#endif  // WEBRTC_WIN

void VPrintError(const char* format, va_list args) {
  vfprintf(stderr, format, args);
}

#if defined(__GNUC__)
extern void PrintError(const char* format, ...)
    __attribute__((__format__(__printf__, 1, 2)));
#endif

void PrintError(const char* format, ...) {
  va_list args;
  va_start(args, format);
  VPrintError(format, args);
  va_end(args);
}

// Function to call from the C version of the RTC_CHECK and RTC_DCHECK macros.
RTC_NORETURN void rtc_FatalMessage(const char* file, int line, const char* msg) {
  fflush(stdout);
  fflush(stderr);
#define FATAL_MESSAGE_FORMAT \
    "\n" \
    "\n" \
    "#\n" \
    "# Fatal error in " "%s" ", line " "%d\n" \
    "# last system error: " "%d\n" \
    "# %s\n" \
    "#\n"
  PrintError(FATAL_MESSAGE_FORMAT, file, line, LAST_SYSTEM_ERROR, msg);
  fflush(stderr);
  abort();
}
