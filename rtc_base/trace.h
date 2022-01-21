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
// #include <stdint.h>
#include <stdio.h> // FILE*

/*
 * int main(void)
 * {
 *   // |WebRtcTrace_SetOStream| calls 
 *   // before |InitFunctionPointers| be called
 *   TraceOStream ostream1 = {
 *     .file = stdout,
 *     .filter = kTraceAll,
 *     .wrap_line_number = 0,
 *   };
 *   WebRtcTrace_SetOStream(&ostream1);
 *   char filename[64];
 *   {
 *     struct timespec ts;
 *     clock_gettime(CLOCK_REALTIME, &ts);
 *     struct tm buffer;
 *     const struct tm *system_time = localtime_r(&ts.tv_sec, &buffer);
 *     snprintf(filename, sizeof(filename), "%s/%s_%02d_%02d_%02d_%02d_%02d_%02d.log",
 *              "/tmp", "webrtc",
 *              system_time->tm_year + 1900, system_time->tm_mon + 1, system_time->tm_mday,
 *              system_time->tm_hour, system_time->tm_min, system_time->tm_sec);
 *   }
 *   FILE* fp = fopen(filename, "wb");
 *   TraceOStream ostream2 = {
 *       .file = fp,
 *       .filter = kTraceError,
 *       .wrap_line_number = 100,
 *   };
 *   WebRtcTrace_SetOStream(&ostream2);
 *   // Test |WEBRTC_TRACE_MAX_OSTREAM_NUMBER|
 *   WebRtcTrace_SetOStream(&ostream2);
 *   WebRtcTrace_SetOStream(&ostream2);
 *   WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, 0xa5,
 *                "failed in TimeToNextUpdate() call");
 *   WEBRTC_TRACE(kTraceWarning, kTraceAudioMixerServer, 0xa5,
 *                "participant must be registered before turning it into anonymous");
 *   // Test |level_filter|
 *   printf("stdout(%p): (%#x), fp(%p): (%#x)\n", 
 *          (void *)stdout, WebRtcTrace_level_filter(stdout), 
 *          (void *)fp, WebRtcTrace_level_filter(fp));
 *   WebRtcTrace_set_level_filter(stdout, kTraceNone);
 *   // Test wrap file and add time
 *   for (size_t i = 0; i < 1024; i++) {
 *       WEBRTC_TRACE(kTraceError, kTraceAudioMixerServer, i,
 *                    "failed in TimeToNextUpdate() call %d", i);
 *       usleep(1 * 1000); // 1ms
 *   }
 *   if (fp) {
 *       fclose(fp);
 *       fp = NULL;
 *   }
 *   return 0;
 * }
 *
 */

#if !defined(NDEBUG) || defined(TRACE_ALWAYS_ON) // #if RTC_DCHECK_IS_ON
#define WEBRTC_TRACE WebRtcTrace_Add
#else
// Disable all TRACE macros. The LOG macro is still functional.
#define WEBRTC_TRACE true ? (void) 0 : WebRtcTrace_Add
#endif // RTC_DCHECK_IS_ON

#if defined(__cplusplus)
extern "C" { // namespace webrtc {
#endif

// From common_types.h
typedef enum {
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
} TraceModule;

// From common_types.h
// Used as bitmask
typedef enum {
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

  kTraceAll = 0xffff,
} TraceLevel;

typedef struct {
  FILE* file; // Used as identifer
  // const char *file_name_utf8;
  TraceLevel filter; // Used as bitmask
  // TraceModule module;
  // wrap file at line number x, so line [x, max) will be wrap written
  // if |wrap_line_number| is GE than max line in file, no more line will be written
  // eg. x is 1000, and max is 1000
  // line [0,1000) will be written once, and no more line will be written
  const size_t wrap_line_number;
  // If |add_file_counter| is false the same file will be
  // reused when it fills up. If it's true a new file with incremented name will be used.
  // const bool add_file_counter;
} TraceOStream;

// Specifies what type of messages should be written to the trace file. The
// |filter| parameter is a bitmask where each message type is enumerated by the
// TraceLevel enumerator. If |file| is nullptr, all ostreams will be changed.
void WebRtcTrace_set_level_filter(FILE* file,
                                  int filter);

// Returns what type of messages are written to the trace file.
int WebRtcTrace_level_filter(FILE* file);

// Registers out stream to receive trace messages.
int WebRtcTrace_SetOStream(TraceOStream* ostream);

// Adds a trace message for writing to file. The message is put in a queue
// for writing to file whenever possible for performance reasons. I.e. there
// is a crash it is possible that the last, vital logs are not logged yet.
// |level| is the type of message to log. If that type of messages is
// filtered it will not be written to file. |module| is an identifier for what
// part of the code the message is coming.
// |id| is an identifier that should be unique for that set of classes that
// are associated (e.g. all instances owned by an engine).
// |msg| and the ellipsis are the same as e.g. snprintf.
void WebRtcTrace_Add(const TraceLevel level,
                     const TraceModule module,
                     const int id,
                     const char* msg, ...)
#if defined(__GNUC__)
    __attribute__((__format__(__printf__, 4, 5)));
#else
    ;
#endif

#if defined(__cplusplus)
} // }  // namespace webrtc
#endif

#endif  // WEBRTC_SYSTEM_WRAPPERS_INCLUDE_TRACE_H_
