/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_SYSTEM_WRAPPERS_SOURCE_TRACE_IMPL_H_
#define WEBRTC_SYSTEM_WRAPPERS_SOURCE_TRACE_IMPL_H_

// combine of m59 webrtc/system_wrappers/source/trace_impl.h
// webrtc/test/testsupport/trace_to_stderr.h and webrtc/system_wrappers/include/file_wrapper.h

#include "rtc_base/trace.h"

#include <memory>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

#define WEBRTC_TRACE_MAX_MESSAGE_SIZE 1024
// Total buffer size is WEBRTC_TRACE_NUM_ARRAY (number of buffer partitions) *
// WEBRTC_TRACE_MAX_QUEUE (number of lines per buffer partition) *
// WEBRTC_TRACE_MAX_MESSAGE_SIZE (number of 1 byte characters per line) =
// 1 or 4 Mbyte.

#define WEBRTC_TRACE_MAX_FILE_SIZE 1000 // 100*1000
// Number of rows that may be written to file. On average 110 bytes per row (max
// 256 bytes per row). So on average 110*100*1000 = 11 Mbyte, max 256*100*1000 =
// 25.6 Mbyte

class TraceImpl final : public Trace {
 public:
  // Singleton, constructor and destructor are private.
  static TraceImpl* Instance();

  bool TraceCheck(const TraceLevel level) const;

 protected:
  /* int SetTraceFileImpl(const char* file_name, const bool add_file_counter); */
  int SetTraceCallbackImpl(TraceCallback* callback);

  void AddImpl(const TraceLevel level, const TraceModule module,
               const int id, const char* msg);

 private:
  friend class Trace;
  TraceImpl();
  virtual ~TraceImpl();

  // OS specific implementations.
  int AddTime(char* trace_message,
              const TraceLevel level) const;

  int AddDateTimeInfo(char* trace_message) const;

  int AddThreadId(char* trace_message) const;

  int AddLevel(char* sz_message, const TraceLevel level) const;

  int AddModuleAndId(char* trace_message, const TraceModule module,
                     const int id) const;

  int AddMessage(char* trace_message,
                 const char msg[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
                 const int written_so_far) const;

  void AddMessageToList(
    const char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
    const int length,
    const TraceLevel level);

  std::unique_ptr<TraceCallback> callback_ RTC_GUARDED_BY(crit_);

  rtc::CriticalSection crit_;

  volatile mutable int64_t prev_api_tick_count_;
  volatile mutable int64_t prev_tick_count_;

  RTC_DISALLOW_COPY_AND_ASSIGN(TraceImpl);
};

}  // namespace webrtc

#endif  // WEBRTC_SYSTEM_WRAPPERS_SOURCE_TRACE_IMPL_H_
