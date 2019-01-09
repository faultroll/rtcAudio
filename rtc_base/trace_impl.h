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

// Based on webrtc/system_wrappers/source/trace_impl.h in owt-deps-webrtc-59-server

// Total buffer size is WEBRTC_TRACE_NUM_ARRAY (number of buffer partitions) *
// WEBRTC_TRACE_MAX_QUEUE (number of lines per buffer partition) *
// WEBRTC_TRACE_MAX_MESSAGE_SIZE (number of 1 byte characters per line) =
// 1 or 4 Mbyte.
#define WEBRTC_TRACE_MAX_MESSAGE_SIZE 1024
// Number of rows that may be written to file. On average 110 bytes per row (max
// 256 bytes per row). So on average 110*100*1000 = 11 Mbyte, max 256*100*1000 =
// 25.6 Mbyte
#define WEBRTC_TRACE_MAX_FILE_SIZE 1000 // 100*1000
// Number of ostream that my be added
#define WEBRTC_TRACE_MAX_OSTREAM_NUMBER 3

#if defined(__cplusplus)
extern "C" { // namespace webrtc {
#endif

// The length of the trace text preceeding the log message.
static const int kBoilerplateLength = 71; // 12(level)+22(time)+25(module,id)+12(tid)
// The length of the timestamp (without "delta" field).
// static const int kTimestampLength = 12;
// The position of the timestamp text within a trace.
// static const int kTimestampPosition = 13; // kTimestampLength+1
// static volatile int level_filter_ = kTraceDefault;

typedef struct {
  TraceOStream ostream;
  size_t row_count_text;
} TraceHandle;

// Form trace message
static void AddImpl(const TraceLevel level,
                    const TraceModule module,
                    const int id,
                    const char* msg);
static int AddLevel(char* trace_message,
                    const TraceLevel level);
static int AddModuleAndId(char* trace_message,
                          const TraceModule module,
                          const int id);
static int AddTraceMessage(char* trace_message,
                           const char* msg,
                           const int written_so_far);
static void AddTraceMessageToList(const char* trace_message,
                                  const int length,
                                  const TraceLevel level);
// OS specific implementations.
static int AddTime(char* trace_message,
                   const TraceLevel level);
static int AddDateTimeInfo(char* trace_message);
static int AddThreadId(char* trace_message);
// Handle functions
static void WriteToFile(TraceHandle *handle,
                        const char* msg,
                        int length);
static bool CheckLevel(TraceHandle *handle,
                       const TraceLevel level);

#if defined(__cplusplus)
} // }  // namespace webrtc
#endif

#endif  // WEBRTC_SYSTEM_WRAPPERS_SOURCE_TRACE_IMPL_H_
