/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 *  System independent wrapper for logging runtime information to file.
 *  Note: All log messages will be written to the same trace file.
 *  Note: If too many messages are written to file there will be a build up of
 *  messages. Apply filtering to avoid that.
 */

#ifndef WEBRTC_SYSTEM_WRAPPERS_INCLUDE_TRACE_H_
#define WEBRTC_SYSTEM_WRAPPERS_INCLUDE_TRACE_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "rtc_base/checks.h"

namespace webrtc {

#if RTC_DCHECK_IS_ON
// Disable all TRACE macros. The LOG macro is still functional.
#define WEBRTC_TRACE true ? (void) 0 : Trace::Add
#else
#define WEBRTC_TRACE Trace::Add
#endif // RTC_DCHECK_IS_ON

// From common_types.h
enum TraceModule {
  kTraceUndefined = 0,
  // not a module, triggered from the engine code
  kTraceVoice = 0x0001,
  // not a module, triggered from the engine code
  kTraceVideo = 0x0002,
  // not a module, triggered from the utility code
  kTraceUtility = 0x0003,
  kTraceRtpRtcp = 0x0004,
  kTraceTransport = 0x0005,
  kTraceSrtp = 0x0006,
  kTraceAudioCoding = 0x0007,
  kTraceAudioMixerServer = 0x0008,
  kTraceAudioMixerClient = 0x0009,
  kTraceFile = 0x000a,
  kTraceAudioProcessing = 0x000b,
  kTraceVideoCoding = 0x0010,
  kTraceVideoMixer = 0x0011,
  kTraceAudioDevice = 0x0012,
  kTraceVideoRenderer = 0x0014,
  kTraceVideoCapture = 0x0015,
  kTraceRemoteBitrateEstimator = 0x0017,
};

enum TraceLevel {
  kTraceNone = 0x0000,  // no trace
  kTraceStateInfo = 0x0001,
  kTraceWarning = 0x0002,
  kTraceError = 0x0004,
  kTraceCritical = 0x0008,
  kTraceApiCall = 0x0010,
  kTraceDefault = 0x00ff,

  kTraceModuleCall = 0x0020,
  kTraceMemory = 0x0100,  // memory info
  kTraceTimer = 0x0200,   // timing info
  kTraceStream = 0x0400,  // "continuous" stream of data

  // used for debug purposes
  kTraceDebug = 0x0800,  // debug
  kTraceInfo = 0x1000,   // debug info

  // Non-verbose level used by LS_INFO of logging.h. Do not use directly.
  kTraceTerseInfo = 0x2000,

  kTraceAll = 0xffff
};

// External Trace API
class TraceCallback {
 public:
  virtual void Print(TraceLevel level, const char* message, int length) = 0;

 protected:
  virtual ~TraceCallback() {}
  TraceCallback() {}
};

class Trace {
 public:
  // The length of the trace text preceeding the log message.
  static const int kBoilerplateLength;
  // The position of the timestamp text within a trace.
  static const int kTimestampPosition;
  // The length of the timestamp (without "delta" field).
  static const int kTimestampLength;

  // Increments the reference count to the trace.
  static void CreateTrace();
  // Decrements the reference count to the trace.
  static void ReturnTrace();
  // Note: any instance that writes to the trace file should increment and
  // decrement the reference count on construction and destruction,
  // respectively.

  // Specifies what type of messages should be written to the trace file. The
  // filter parameter is a bitmask where each message type is enumerated by the
  // TraceLevel enumerator. TODO(hellner): why is the TraceLevel enumerator not
  // defined in this file?
  static void set_level_filter(int filter);

  // Returns what type of messages are written to the trace file.
  static int level_filter();

  // Sets the file name. If add_file_counter is false the same file will be
  // reused when it fills up. If it's true a new file with incremented name
  // will be used.
  static int32_t SetTraceFile(const char* file_name,
                              const bool add_file_counter = false);

  // Registers callback to receive trace messages.
  // TODO(hellner): Why not use OutStream instead? Why is TraceCallback not
  // defined in this file?
  static int32_t SetTraceCallback(TraceCallback* callback);

  // Adds a trace message for writing to file. The message is put in a queue
  // for writing to file whenever possible for performance reasons. I.e. there
  // is a crash it is possible that the last, vital logs are not logged yet.
  // level is the type of message to log. If that type of messages is
  // filtered it will not be written to file. module is an identifier for what
  // part of the code the message is coming.
  // id is an identifier that should be unique for that set of classes that
  // are associated (e.g. all instances owned by an engine).
  // msg and the ellipsis are the same as e.g. sprintf.
  // TODO(hellner) Why is TraceModule not defined in this file?
  static void Add(const TraceLevel level,
                  const TraceModule module,
                  const int32_t id,
                  const char* msg, ...);

 private:
  static volatile int level_filter_;
};

}  // namespace webrtc

#endif  // WEBRTC_SYSTEM_WRAPPERS_INCLUDE_TRACE_H_
