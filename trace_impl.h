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

#include "rtc_base/trace.h"

// #include <memory>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

#define WEBRTC_TRACE_MAX_MESSAGE_SIZE 1024
// Total buffer size is WEBRTC_TRACE_NUM_ARRAY (number of buffer partitions) *
// WEBRTC_TRACE_MAX_QUEUE (number of lines per buffer partition) *
// WEBRTC_TRACE_MAX_MESSAGE_SIZE (number of 1 byte charachters per line) =
// 1 or 4 Mbyte.

/* #define WEBRTC_TRACE_MAX_FILE_SIZE 100*1000 */
// Number of rows that may be written to file. On average 110 bytes per row (max
// 256 bytes per row). So on average 110*100*1000 = 11 Mbyte, max 256*100*1000 =
// 25.6 Mbyte

class TraceImpl final : public Trace {
 public:
  // Singleton, constructor and destructor are private.
  static TraceImpl* Instance();

  /* int32_t SetTraceFileImpl(const char* file_name, const bool add_file_counter); */
  int32_t SetTraceCallbackImpl(TraceCallback* callback);

  void AddImpl(const TraceLevel level, const TraceModule module,
               const int32_t id, const char* msg);

  bool TraceCheck(const TraceLevel level) const;

 protected:



 private:
  // friend class Trace;
  TraceImpl();
  virtual ~TraceImpl();

  // OS specific implementations.
  int32_t AddTime(char* trace_message,
                          const TraceLevel level) const;

  int32_t AddDateTimeInfo(char* trace_message) const;

  int32_t AddThreadId(char* trace_message) const;

  int32_t AddLevel(char* sz_message, const TraceLevel level) const;

  int32_t AddModuleAndId(char* trace_message, const TraceModule module,
                         const int32_t id) const;

  int32_t AddMessage(char* trace_message,
                     const char msg[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
                     const uint16_t written_so_far) const;

  void AddMessageToList(
    const char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
    const uint16_t length,
    const TraceLevel level);

  /* bool UpdateFileName(
      char file_name_with_counter_utf8[FileWrapper::kMaxFileNameSize],
      const uint32_t new_count) const EXCLUSIVE_LOCKS_REQUIRED(crit_);

  bool CreateFileName(
    const char file_name_utf8[FileWrapper::kMaxFileNameSize],
    char file_name_with_counter_utf8[FileWrapper::kMaxFileNameSize],
    const uint32_t new_count) const;

  void WriteToFile(const char* msg, uint16_t length)
      EXCLUSIVE_LOCKS_REQUIRED(crit_); */

  TraceCallback* callback_ RTC_GUARDED_BY(crit_);
  /* uint32_t row_count_text_ RTC_GUARDED_BY(crit_);
  uint32_t file_count_text_ RTC_GUARDED_BY(crit_); */

  /* const std::unique_ptr<FileWrapper> trace_file_ RTC_GUARDED_BY(crit_);
  std::string trace_file_path_ RTC_GUARDED_BY(crit_); */
  rtc::CriticalSection crit_;

  bool is_stderr_;

  volatile mutable int64_t prev_api_tick_count_;
  volatile mutable int64_t prev_tick_count_;

  RTC_DISALLOW_COPY_AND_ASSIGN(TraceImpl);
};

}  // namespace webrtc

#endif  // WEBRTC_SYSTEM_WRAPPERS_SOURCE_TRACE_IMPL_H_
