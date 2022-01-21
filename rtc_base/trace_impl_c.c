/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/trace_impl.h"

#include <assert.h> // should be first
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static struct {
  size_t handle_number;
  TraceHandle handles[WEBRTC_TRACE_MAX_OSTREAM_NUMBER];
} trace_impl_ = {0};
// var, head, field
#define HANDLE_FOREACH(var)    \
  var = &trace_impl_.handles[0]; \
  for (size_t _i = 0; \
       _i < trace_impl_.handle_number; \
       ++_i, var = &trace_impl_.handles[_i])

// |dw_delta_time| is meanless with different module and id
// we need a map {(module << 16 | id), prev_tick_count} to make this work
#if 0
static void once(void (*func)(void));
static inline 
int64_t UpdateTickCount(const TraceLevel level,
                        int64_t dw_current_time) {
  // TODO(lgY): lock
  static int64_t prev_tick_count_ = -1, prev_api_tick_count_ = -1;
  // TODO(lgY): Use once
  if (-1 == prev_tick_count_ && -1 == prev_api_tick_count_) {
    prev_tick_count_ = prev_api_tick_count_ = dw_current_time;
    return 0;
  }
  int64_t dw_delta_time;
  if (level != kTraceApiCall) {
    dw_delta_time = dw_current_time - prev_tick_count_;
    prev_tick_count_ = dw_current_time;
  } else {
    dw_delta_time = dw_current_time - prev_api_tick_count_;
    prev_api_tick_count_ = dw_current_time;
  }
  return dw_delta_time;
}
#endif

#if defined(WEBRTC_WIN)

#ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
#endif /* WIN32_LEAN_AND_MEAN */
#include <windows.h>
#include <mmsystem.h>
#include <sys/timeb.h>
#pragma warning(disable:4355)
#ifndef TIME_UTC
    #define TIME_UTC 1
#endif /* TIME_UTC */
#ifndef _STRUCT_TIMESPEC
    struct timespec {
        time_t tv_sec;
        long   tv_nsec;
    };
    #define _STRUCT_TIMESPEC
#endif /* _STRUCT_TIMESPEC */

static
int AddTime(char* trace_message,
            const TraceLevel level) {
#if 0
  const int kMessageLength = 22;
  // tick
  int64_t dw_current_time, dw_delta_time;
  // struct timespec ts;
  // CLOCK_MONOTONIC
  /* {
    int64_t ticks;
    static volatile LONG last_timegettime = 0;
    static volatile int64_t num_wrap_timegettime = 0;
    volatile LONG *last_timegettime_ptr = &last_timegettime;
    DWORD now = timeGetTime();
    // Atomically update the last gotten time
    DWORD old = InterlockedExchange(last_timegettime_ptr, now);
    if (now < old) {
      // If now is earlier than old, there may have been a race between threads.
      // 0x0fffffff ~3.1 days, the code will not take that long to execute
      // so it must have been a wrap around.
      if (old > 0xf0000000 && now < 0x0fffffff) {
        num_wrap_timegettime++;
      }
    }
    ticks = now + (num_wrap_timegettime << 32);
    // TODO(deadbeef): Calculate with nanosecond precision. Otherwise, we're
    // just wasting a multiply and divide when doing Time() on Windows.
    // ts.tv_sec  = ticks / INT64_C(1000);
    // ts.tv_nsec = (ticks % INT64_C(1000)) * INT64_C(1000000);
    dw_current_time = ticks;
  } */
  // CLOCK_REALTIME
  {
    struct _timeb time;
    _ftime(&time);
    // Convert from second (1.0) and milliseconds (1e-3).
    // ts->tv_sec = (int64_t)(time.time);
    // ts->tv_nsec = (int64_t)(time.millitm) * INT64_C(1000000);
    dw_current_time = (int64_t)(time.time) * INT64_C(1000) + 
                      (int64_t)(time.millitm);
  }
  dw_delta_time = UpdateTickCount(level, dw_current_time);
  // tm
  SYSTEMTIME system_time;
  GetSystemTime(&system_time);
  // combine
  sprintf(trace_message, "(%2u:%2u:%2u:%3u |%5lu) ", system_time.wHour,
          system_time.wMinute, system_time.wSecond, system_time.wMilliseconds, 
          (long)(dw_delta_time));
  // Messages are 22 characters.
  return kMessageLength;
#else
  const int kMessageLength = 15;
  // tm
  SYSTEMTIME system_time;
  GetSystemTime(&system_time);
  sprintf(trace_message, "(%2u:%2u:%2u:%3u) ", system_time.wHour,
          system_time.wMinute, system_time.wSecond, system_time.wMilliseconds);
  // Messages are 17 characters.
  return kMessageLength;
#endif
}

static
int AddDateTimeInfo(char* trace_message) {
  // datetime
  TCHAR sz_date_str[20];
  TCHAR sz_time_str[20];
  SYSTEMTIME sys_time;
  GetLocalTime(&sys_time);
  // Create date string (e.g. Apr 04 2002)
  GetDateFormat(LOCALE_SYSTEM_DEFAULT, 0, &sys_time, TEXT("MMM dd yyyy"),
                sz_date_str, 20);
  // Create time string (e.g. 15:32:08)
  GetTimeFormat(LOCALE_SYSTEM_DEFAULT, 0, &sys_time, TEXT("HH':'mm':'ss"),
                sz_time_str, 20);
  snprintf(trace_message, WEBRTC_TRACE_MAX_MESSAGE_SIZE, 
           "Local Date: %ls Local Time: %ls", sz_date_str, sz_time_str);
  // Include NULL termination (hence + 1).
  return strlen(trace_message) + 1;
}

static 
int AddThreadId(char* trace_message) {
  // const int kMessageLength = 12;
  unsigned long thread_id = (unsigned long)(GetCurrentThread());
  // Messages is 12 characters.
  return sprintf(trace_message, "%10lu; ", thread_id);
}

// From common_audio/signal_processing/spl_init.c
static 
void once(void (*func)(void)) {
  /* Didn't use InitializeCriticalSection() since there's no race-free context
   * in which to execute it.
   *
   * TODO(kma): Change to different implementation (e.g.
   * InterlockedCompareExchangePointer) to avoid issues similar to
   * http://code.google.com/p/webm/issues/detail?id=467.
   */
  static CRITICAL_SECTION lock = {(void *)((size_t)-1), -1, 0, 0, 0, 0};
  static int done = 0;

  EnterCriticalSection(&lock);
  if (!done) {
    func();
    done = 1;
  }
  LeaveCriticalSection(&lock);
}

#else // WEBRTC_WIN

#include <time.h>
// #include <sys/time.h>
#include <pthread.h>

static
int AddTime(char* trace_message,
            const TraceLevel level) {
#if 0
  const int kMessageLength = 22;
  // tick
  int64_t dw_current_time, dw_delta_time;
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts); // CLOCK_MONOTONIC
  dw_current_time = (int64_t)(ts.tv_sec) * INT64_C(1000) + 
                    (int64_t)(ts.tv_nsec) / INT64_C(1000000);
  dw_delta_time = UpdateTickCount(level, dw_current_time);
  // tm
  struct tm buffer;
  const struct tm* system_time = localtime_r(&ts.tv_sec, &buffer);
  const int64_t ms_time = (int64_t)(ts.tv_nsec) / INT64_C(1000000);
  // combine
  sprintf(trace_message, "(%2u:%2u:%2u:%3lu |%5lu) ", system_time->tm_hour,
          system_time->tm_min, system_time->tm_sec, (long)(ms_time), 
          (long)(dw_delta_time));
  // Messages are 22 characters.
  return kMessageLength;
#else
  const int kMessageLength = 15;
  // tm
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm buffer;
  const struct tm* system_time = localtime_r(&ts.tv_sec, &buffer);
  const int64_t ms_time = (int64_t)(ts.tv_nsec) / INT64_C(1000000);
  // combine
  sprintf(trace_message, "(%2u:%2u:%2u:%3lu) ", system_time->tm_hour,
          system_time->tm_min, system_time->tm_sec, (long)(ms_time));
  // Messages are 17 characters.
  return kMessageLength;
#endif
}

static
int AddDateTimeInfo(char* trace_message) {
  // datetime
  time_t t;
  time(&t);
  char buffer[26];  // man ctime says buffer should have room for >=26 bytes.
  snprintf(trace_message, WEBRTC_TRACE_MAX_MESSAGE_SIZE, 
          "Local DateTime: %s", ctime_r(&t, buffer));
  int len = (int)(strlen(trace_message));
  if ('\n' == trace_message[len - 1]) {
    trace_message[len - 1] = '\0';
    --len;
  }
  return len + 1;
}

static 
int AddThreadId(char* trace_message) {
  // const int kMessageLength = 12;
  unsigned long thread_id = (unsigned long)(pthread_self());
  // Messages is 12 characters.
  return sprintf(trace_message, "%10lu; ", thread_id);
}

static 
void once(void (*func)(void)) {
  static pthread_once_t lock = PTHREAD_ONCE_INIT;
  pthread_once(&lock, func);
}

#endif // WEBRTC_WIN

static 
int AddLevel(char* trace_message,
             const TraceLevel level) {
  const int kMessageLength = 12;
  switch (level) {
    case kTraceStateInfo:
      sprintf(trace_message, "STATEINFO ; ");
      break;
    case kTraceWarning:
      sprintf(trace_message, "WARNING   ; ");
      break;
    case kTraceError:
      sprintf(trace_message, "ERROR     ; ");
      break;
    case kTraceCritical:
      sprintf(trace_message, "CRITICAL  ; ");
      break;
    case kTraceApiCall:
      sprintf(trace_message, "APICALL   ; ");
      break;
    case kTraceModuleCall:
      sprintf(trace_message, "MODULECALL; ");
      break;
    case kTraceMemory:
      sprintf(trace_message, "MEMORY    ; ");
      break;
    case kTraceTimer:
      sprintf(trace_message, "TIMER     ; ");
      break;
    case kTraceStream:
      sprintf(trace_message, "STREAM    ; ");
      break;
    case kTraceDebug:
      sprintf(trace_message, "DEBUG     ; ");
      break;
    case kTraceInfo:
      sprintf(trace_message, "DEBUGINFO ; ");
      break;
    // case kTraceTerseInfo:
    // case kTraceDefault:
    // case kTraceAll:
    default:
      /* // Add the appropriate amount of whitespace.
      memset(trace_message, ' ', kMessageLength);
      trace_message[kMessageLength] = '\0';
      break; */
      // Skip this, so length is 0
      return 0;
  }
  // All messages are 12 characters.
  return kMessageLength;
}

static
int AddModuleAndId(char* trace_message,
                   const TraceModule module,
                   const int id) {
  const int kMessageLength = 25;
  // Use long int to prevent problems with different definitions of
  // int.
  // TODO(hellner): is this actually a problem? If so, it should be better to
  //                clean up int
  const long idl = id;
  // const unsigned long int id_engine = id >> 16;
  // const unsigned long int id_channel = id & 0xffff;
  // sprintf(trace_message, "       VOICE:%5ld %5ld;", id_engine, id_channel);
  switch (module) {
    /* case kTraceUndefined:
      // Add the appropriate amount of whitespace.
      memset(trace_message, ' ', kMessageLength);
      trace_message[kMessageLength] = '\0';
      break; */
    case kTraceVoice:
      sprintf(trace_message, "       VOICE:%11ld;", idl);
      break;
    case kTraceVideo:
      sprintf(trace_message, "       VIDEO:%11ld;", idl);
      break;
    case kTraceUtility:
      sprintf(trace_message, "     UTILITY:%11ld;", idl);
      break;
    case kTraceRtpRtcp:
      sprintf(trace_message, "    RTP/RTCP:%11ld;", idl);
      break;
    case kTraceTransport:
      sprintf(trace_message, "   TRANSPORT:%11ld;", idl);
      break;
    case kTraceSrtp:
      sprintf(trace_message, "        SRTP:%11ld;", idl);
      break;
    case kTraceAudioCoding:
      sprintf(trace_message, "AUDIO CODING:%11ld;", idl);
      break;
    case kTraceAudioMixerServer:
      sprintf(trace_message, " AUDIO MIX/S:%11ld;", idl);
      break;
    case kTraceAudioMixerClient:
      sprintf(trace_message, " AUDIO MIX/C:%11ld;", idl);
      break;
    case kTraceFile:
      sprintf(trace_message, "        FILE:%11ld;", idl);
      break;
    case kTraceAudioProcessing:
      sprintf(trace_message, "  AUDIO PROC:%11ld;", idl);
      break;
    case kTraceVideoCoding:
      sprintf(trace_message, "VIDEO CODING:%11ld;", idl);
      break;
    case kTraceVideoMixer:
      sprintf(trace_message, "   VIDEO MIX:%11ld;", idl);
      break;
    case kTraceAudioDevice:
      sprintf(trace_message, "AUDIO DEVICE:%11ld;", idl);
      break;
    case kTraceVideoRenderer:
      sprintf(trace_message, "VIDEO RENDER:%11ld;", idl);
      break;
    case kTraceVideoCapture:
      sprintf(trace_message, "VIDEO CAPTUR:%11ld;", idl);
      break;
    case kTraceRemoteBitrateEstimator:
      sprintf(trace_message, "     BWE RBE:%11ld;", idl);
      break;
    // case kTraceUndefined:
    default:
      /* // Cannot happen
      assert(false);
      return -1; */
      // Skip this, so length is 0
      return 0;
  }
  // All messages are 25 characters.
  return kMessageLength;
}

static 
int AddTraceMessage(char* trace_message,
                    const char* msg,
                    const int written_so_far) {
  const int length_remain = WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2;
  int length = 0;
  if (written_so_far >= WEBRTC_TRACE_MAX_MESSAGE_SIZE) {
    return -1;
  }
  // - 2 to leave room for newline and NULL termination.
  length = snprintf(trace_message, length_remain, "%s", msg);
  if (length < 0 || length > length_remain) {
    length = length_remain;
    trace_message[length] = '\0';
  }
  // Length with NULL termination.
  return length + 1;
}

static 
void AddTraceMessageToList(const char* trace_message,
                           const int length,
                           const TraceLevel level) {
  // Print to all ostreams
  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    if (CheckLevel(handle, level)) {
      WriteToFile(handle, trace_message, length);
    }
  }
}

static 
void AddImpl(const TraceLevel level,
                    const TraceModule module,
                    const int id,
                    const char* msg) {
  char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
  char* message_ptr = &trace_message[0];
  int len = -1, ack_len = 0;

  len = AddLevel(message_ptr, level);
  if (len == -1)
    return;
  message_ptr += len;
  ack_len += len;

  len = AddTime(message_ptr, level);
  if (len == -1)
    return;
  message_ptr += len;
  ack_len += len;

  len = AddModuleAndId(message_ptr, module, id);
  if (len == -1)
    return;
  message_ptr += len;
  ack_len += len;

  len = AddThreadId(message_ptr);
  if (len < 0)
    return;
  message_ptr += len;
  ack_len += len;

  len = AddTraceMessage(message_ptr, msg, ack_len);
  if (len == -1)
    return;
  // message_ptr += len;
  ack_len += len;

  trace_message[ack_len] = '\0';
  trace_message[ack_len - 1] = '\n';
  AddTraceMessageToList(trace_message, ack_len, level);
}

// CreateFileName is same as UpdateFileName
/* static inline 
bool UpdateFileName(const char* file_name_utf8,
                    char* file_name_with_counter_utf8,
                    const size_t new_count) {
  // TODO(lgY): check file_name_utf8 
  size_t length = strlen(file_name_utf8);
  if (length == 0) {
    return false;
  }
  // find suffix
  size_t length_without_file_ending = length;
  do {
    if (file_name_utf8[length_without_file_ending] == '.') {
      break;
    } else {
      length_without_file_ending--;
    }
  } while (length_without_file_ending > 0);
  if (length_without_file_ending == 0) {
    length_without_file_ending = length;
  }
  // find count
  size_t length_to = length_without_file_ending;
  do {
    if (file_name_utf8[length_to] == '_') {
      break;
    } else {
      length_to--;
    }
  } while (length_to > 0);
 if (length_to == 0) {
    length_to = length_without_file_ending;
  }

  memcpy(&file_name_with_counter_utf8[0], &file_name_utf8[0], length_to);
  sprintf(&file_name_with_counter_utf8[length_to], "_%lu%s",
          (unsigned long)(new_count),
          &file_name_utf8[length_without_file_ending]);
  return true;
} */

static
void WriteToFile(TraceHandle *handle,
                 const char* msg,
                 int length) {
  if (!handle->ostream.file)
    return;

  // if (handle->wrap_line_number > WEBRTC_TRACE_MAX_FILE_SIZE)
  //    // no wrap will happen
  // else // if (handle->wrap_line_number =< WEBRTC_TRACE_MAX_FILE_SIZE)
  //    // line [handle->wrap_line_number, WEBRTC_TRACE_MAX_FILE_SIZE)
  //    // will be wrap wriiten
  if (handle->row_count_text >= handle->ostream.wrap_line_number) {
    if (handle->row_count_text == handle->ostream.wrap_line_number) {
      handle->wrap_offset = ftell(handle->ostream.file);
    }
    if (handle->row_count_text >= WEBRTC_TRACE_MAX_FILE_SIZE) {
      /* if (handle->file_count_text == 0) */ {
        // wrap file
        handle->row_count_text = handle->ostream.wrap_line_number;
        // fdatasync(fileno(handle->ostream.file));
        fseek(handle->ostream.file, handle->wrap_offset, SEEK_SET);
      } /* else {
        char new_file_name[WEBRTC_TRACE_MAX_FILENAME_SIZE];
        // get current name
        handle->file_count_text++;
        UpdateFileName(handle->ostream.file_name_utf8, new_file_name,
                       handle->file_count_text);
        // TODO(lgY): race here if file closed in one thread 
        // but other thread(s) does not stop writting file 
        fclose(handle->ostream.file);
        handle->ostream.file = fopen(new_file_name, "wb");
      } */
    }
  }
  if (handle->row_count_text < WEBRTC_TRACE_MAX_FILE_SIZE) {
    fwrite(msg, 1, length, handle->ostream.file);
    fflush(handle->ostream.file);
    handle->row_count_text++;
  }
}

static 
bool CheckLevel(TraceHandle *handle,
                const TraceLevel level) {
  return ((level & handle->ostream.filter)) ? true : false;
}

static inline 
bool TraceCheck(const TraceLevel level) {
  // Check all ostreams
  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    if (CheckLevel(handle, level)) {
        return true;
    }
  }
  return false;
}

static inline 
void TraceInit(void) {
  // If no ostream, add default ostream
  if (!trace_impl_.handle_number) {
    TraceOStream ostream = {
      .file = stdout,
      .filter = kTraceDefault,
      .wrap_line_number = 0,
    };
    WebRtcTrace_SetOStream(&ostream);
  }
  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    char msg_tmp[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
    int length_tmp = AddDateTimeInfo(msg_tmp);
    msg_tmp[length_tmp] = '\0';
    msg_tmp[length_tmp - 1] = '\n';
    fwrite(msg_tmp, 1, length_tmp, handle->ostream.file);
    fflush(handle->ostream.file);
  }
}

void WebRtcTrace_set_level_filter(FILE* file,
                                  int filter) {
  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    if (!file || (file == handle->ostream.file)) {
      handle->ostream.filter = filter;
    }
  }
}

int WebRtcTrace_level_filter(FILE* file) {
  if (!file)
    return -1;

  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    if (file == handle->ostream.file) {
      return handle->ostream.filter;
    }
  }
  return -1;
}

int WebRtcTrace_SetOStream(TraceOStream* ostream) {
  if (trace_impl_.handle_number >= WEBRTC_TRACE_MAX_OSTREAM_NUMBER ||
      NULL == ostream || !ostream->file) {
    return -1;
  }
  TraceHandle* handle;
  HANDLE_FOREACH(handle) {
    if (ostream->file == handle->ostream.file) {
      return -1;
    }
  }

  // TODO(lgY): lock or atomic_add |handle_number|
  memcpy(&trace_impl_.handles[trace_impl_.handle_number].ostream, 
         ostream, 
         sizeof(TraceOStream));
  trace_impl_.handle_number++;
  return 0;
}

void WebRtcTrace_Add(const TraceLevel level,
                     const TraceModule module,
                     const int id,
                     const char* msg, ...) {
  once(TraceInit);
  if (TraceCheck(level)) {
    char buff_data[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
    const char* buff = buff_data; // RTC_VIEW(const char*)
    if (msg) {
      va_list args;
      va_start(args, msg);
      vsnprintf(buff_data, WEBRTC_TRACE_MAX_MESSAGE_SIZE - 1, msg, args);
      va_end(args);
    }
    AddImpl(level, module, id, buff);
  }
}
