/*
 *  Copyright 2006 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Most of this was borrowed (with minor modifications) from V8's and Chromium's
// src/base/logging.cc.

#if defined(WEBRTC_WIN)
#include <windows.h>
#endif

#include "rtc_base/checks.h"

#if defined(_MSC_VER)
// Warning C4722: destructor never returns, potential memory leak.
// FatalMessage's dtor very intentionally aborts.
#pragma warning(disable:4722)
#endif

namespace rtc {

FatalMessage::FatalMessage(const char* file, int line)
  : file_(file),
    line_(line) {
}

FatalMessage::FatalMessage(const char* file, int line, std::string* result)
  : file_(file),
    line_(line) {
  stream_ << "Check failed: " << *result << std::endl << "# ";
  delete result;
}

RTC_NORETURN FatalMessage::~FatalMessage() {
  rtc_FatalMessage(file_.c_str(), line_, stream_.str().c_str());
}

}  // namespace rtc

