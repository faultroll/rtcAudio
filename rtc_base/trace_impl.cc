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

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "rtc_base/atomic_ops.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/time_utils.h"
#if defined(WEBRTC_WIN)
#include <mmsystem.h>
#else
#include <sys/time.h>
#include <time.h>
#endif // WEBRTC_WIN

#if defined(WEBRTC_WIN)
#pragma warning(disable:4355)
#endif // WEBRTC_WIN

namespace webrtc {

const int Trace::kBoilerplateLength = 71;
const int Trace::kTimestampPosition = 13;
const int Trace::kTimestampLength = 12;
volatile int Trace::level_filter_ = kTraceDefault;

namespace {

// Upon constructing an instance of this class, all traces will be redirected
// to stderr. At destruction, redirection is halted.
class TraceToStderr : public TraceCallback {
 public:
  static const int kLevelFilter = kTraceError | kTraceWarning | kTraceTerseInfo;

  TraceToStderr();
  // Set |override_time| to true to control the time printed with each trace
  // through SetTimeSeconds(). Otherwise, the trace's usual wallclock time is
  // used.
  //
  // This is useful for offline test tools, where the file time is much more
  // informative than the real time.
  explicit TraceToStderr(bool override_time);
  ~TraceToStderr() override;

  // Every subsequent trace printout will use |time|. Has no effect if
  // |override_time| in the constructor was set to false.
  //
  // No attempt is made to ensure thread-safety between the trace writing and
  // time updating. In tests, since traces will normally be triggered by the
  // main thread doing the time updating, this should be of no concern.
  virtual void SetTimeSeconds(float time);

 public:
  // Implements TraceCallback.
  void Print(TraceLevel level, const char* msg, int length) override;
 private:
  bool override_time_;
  float time_seconds_;

  class FileWrapper {
   public:
    static const size_t kMaxFileNameSize = 1024;

    FileWrapper() {}
    ~FileWrapper() { CloseFile(); }

    // Returns true if a file has been opened.
    bool is_open() const { return file_ != nullptr; }
    // Opens a file in read or write mode, decided by the read_only parameter.
    bool OpenFile(const char* file_name_utf8, bool read_only) {
      size_t length = strlen(file_name_utf8);
      if (length > kMaxFileNameSize - 1)
        return false;
      if (file_ != nullptr)
        return false;
    #if defined(WEBRTC_WIN)
      int len = MultiByteToWideChar(CP_UTF8, 0, file_name_utf8, -1, nullptr, 0);
      std::wstring wstr(len, 0);
      MultiByteToWideChar(CP_UTF8, 0, file_name_utf8, -1, &wstr[0], len);
      file_ = _wfopen(wstr.c_str(), read_only ? L"rb" : L"wb");
    #else
      file_ = fopen(file_name_utf8, read_only ? "rb" : "wb");
    #endif
      return file_ != nullptr;
    }
    void CloseFile() {
      if (file_ != nullptr)
        fclose(file_);
      file_ = nullptr;
    }
    // Flush any pending writes.  Note: Flushing when closing, is not required.
    int Flush() {
      return (file_ != nullptr) ? fflush(file_) : -1;
    }
    // Rewinds the file to the start.
    int Rewind() {
      if (file_ != nullptr) {
        position_ = 0;
        return fseek(file_, 0, SEEK_SET);
      }
      return -1;
    }
    int Read(void* buf, size_t length) {
      if (file_ == nullptr)
        return -1;
      size_t bytes_read = fread(buf, 1, length, file_);
      return static_cast<int>(bytes_read);
    }
    bool Write(const void* buf, size_t length) {
      if (buf == nullptr)
        return false;
      if (file_ == nullptr)
        return false;
      size_t num_bytes = fwrite(buf, 1, length, file_);
      position_ += num_bytes;
      return num_bytes == length;
    }

   private:
    FILE* file_ = nullptr;
    size_t position_ = 0;

    RTC_DISALLOW_COPY_AND_ASSIGN(FileWrapper);
  };

 public:
  void WriteToFile(const char* message, int length);
 private:
  FILE* OpenFile(const char* file_name_utf8, bool read_only);
  void CloseFile();

  size_t row_count_text_;
  size_t file_count_text_;
  const std::unique_ptr<FileWrapper> trace_file_;
  std::string trace_file_path_;
};

TraceToStderr::TraceToStderr()
    : override_time_(false),
      time_seconds_(0),
      row_count_text_(0),
      file_count_text_(0),
      trace_file_(new FileWrapper()),
      trace_file_path_("/tmp/rtc_trace") {
  trace_file_->OpenFile(trace_file_path_.c_str(), false);
  Trace::set_level_filter(kLevelFilter);
  Trace::CreateTrace();
  Trace::SetTraceCallback(this);
}

TraceToStderr::TraceToStderr(bool override_time)
    : override_time_(override_time),
      time_seconds_(0) {
  Trace::set_level_filter(kLevelFilter);
  Trace::CreateTrace();
  Trace::SetTraceCallback(this);
}

TraceToStderr::~TraceToStderr() {
  Trace::SetTraceCallback(NULL);
  Trace::ReturnTrace();
}

void TraceToStderr::SetTimeSeconds(float time) { time_seconds_ = time; }

void TraceToStderr::Print(TraceLevel level, const char* msg, int length) {
  if (level & kLevelFilter) {
    assert(length > Trace::kBoilerplateLength);
    std::string msg = msg;
    std::string msg_log = msg.substr(Trace::kBoilerplateLength);
    if (override_time_) {
      fprintf(stderr, "%.2fs %s\n", time_seconds_, msg_log.c_str());
    } else {
      std::string msg_time = msg.substr(Trace::kTimestampPosition,
                                        Trace::kTimestampLength);
      fprintf(stderr, "%s %s\n", msg_time.c_str(), msg_log.c_str());
    }
    fflush(stderr);
  }
}

void TraceToStderr::WriteToFile(const char* msg, int length) {
  if (!trace_file_->is_open())
    return;

  if (row_count_text_ > WEBRTC_TRACE_MAX_FILE_SIZE) {
    // wrap file
    row_count_text_ = 0;
    trace_file_->Flush();

    if (file_count_text_ == 0) {
      trace_file_->Rewind();
    } /* else {
      char new_file_name[FileWrapper::kMaxFileNameSize];

      // get current name
      file_count_text_++;
      UpdateFileName(new_file_name, file_count_text_);

      trace_file_->CloseFile();
      trace_file_path_.clear();

      if (!trace_file_->OpenFile(new_file_name, false)) {
        return;
      }
      trace_file_path_ = new_file_name;
    }
  } */
  /* if (row_count_text_ == 0) {
    char message[WEBRTC_TRACE_MAX_MESSAGE_SIZE + 1];
    int length = AddDateTimeInfo(message);
    if (length != -1) {
      message[length] = 0;
      message[length - 1] = '\n';
      trace_file_->Write(message, length);
      row_count_text_++;
    } */
  }

  char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
  memcpy(trace_message, msg, length);
  trace_message[length] = 0;
  trace_message[length - 1] = '\n';
  trace_file_->Write(trace_message, length);
  row_count_text_++;
}

/* bool TraceImpl::UpdateFileName(
    char file_name_with_counter_utf8[FileWrapper::kMaxFileNameSize],
    const size_t new_count) const {
  int length = trace_file_path_.length();

  int length_without_file_ending = length - 1;
  while (length_without_file_ending > 0) {
    if (trace_file_path_[length_without_file_ending] == '.') {
      break;
    } else {
      length_without_file_ending--;
    }
  }
  if (length_without_file_ending == 0) {
    length_without_file_ending = length;
  }
  int length_to_ = length_without_file_ending - 1;
  while (length_to_ > 0) {
    if (trace_file_path_[length_to_] == '_') {
      break;
    } else {
      length_to_--;
    }
  }

  memcpy(file_name_with_counter_utf8, &trace_file_path_[0], length_to_);
  sprintf(file_name_with_counter_utf8 + length_to_, "_%lu%s",
          static_cast<long unsigned int>(new_count),
          &trace_file_path_[length_without_file_ending]);
  return true;
} */

/* bool TraceImpl::CreateFileName(
    const char file_name_utf8[FileWrapper::kMaxFileNameSize],
    char file_name_with_counter_utf8[FileWrapper::kMaxFileNameSize],
    const size_t new_count) const {
  int length = strlen(file_name_utf8);
  if (length < 0) {
    return false;
  }

  int length_without_file_ending = length - 1;
  while (length_without_file_ending > 0) {
    if (file_name_utf8[length_without_file_ending] == '.') {
      break;
    } else {
      length_without_file_ending--;
    }
  }
  if (length_without_file_ending == 0) {
    length_without_file_ending = length;
  }
  memcpy(file_name_with_counter_utf8, file_name_utf8,
         length_without_file_ending);
  sprintf(file_name_with_counter_utf8 + length_without_file_ending, "_%lu%s",
          static_cast<long unsigned int>(new_count),
          file_name_utf8 + length_without_file_ending);
  return true;
} */

/* int TraceImpl::SetTraceFileImpl(const char* file_name_utf8,
                                   const bool add_file_counter) {
  rtc::CritScope lock(&crit_);

  trace_file_->CloseFile();
  trace_file_path_.clear();

  if (file_name_utf8) {
    is_stderr_ = false;

    if (add_file_counter) {
      file_count_text_ = 1;

      char file_name_with_counter_utf8[FileWrapper::kMaxFileNameSize];
      CreateFileName(file_name_utf8, file_name_with_counter_utf8,
                     file_count_text_);
      if (!trace_file_->OpenFile(file_name_with_counter_utf8, false)) {
        return -1;
      }
      trace_file_path_ = file_name_with_counter_utf8;
    } else {
      file_count_text_ = 0;
      if (!trace_file_->OpenFile(file_name_utf8, false)) {
        return -1;
      }
      trace_file_path_ = file_name_utf8;
    }
  } else {
    is_stderr_ = true;

    if (!trace_file_->OpenFromFileHandle(stderr)) {
      return -1;
    }
  }
  row_count_text_ = 0;
  return 0;
} */

}  // namespace

TraceImpl::TraceImpl()
    : callback_(new TraceToStderr()) {
  prev_api_tick_count_ = prev_tick_count_ = 0;
}

TraceImpl::~TraceImpl() {
}

int TraceImpl::AddThreadId(char* trace_message) const {
  size_t thread_id = (size_t)rtc::ThrdCurrent();
  // Messages is 12 characters.
  return sprintf(trace_message, "%10zu; ", thread_id);
}

#if defined(WEBRTC_WIN)
int TraceImpl::AddTime(char* trace_message,
                           const TraceLevel level) const {
  // tick
  struct timespec ts;
  timespec_get_systime(&ts);
  int64_t dw_current_time = timespec_to_millisec(&ts);
  int64_t dw_delta_time;
  if (level == kTraceApiCall) {
    dw_delta_time = dw_current_time - prev_tick_count_;
    prev_tick_count_ = dw_current_time;
    /* if (prev_tick_count_ == 0) {
      dw_delta_time = 0;
    } */
  } else {
    dw_delta_time = dw_current_time - prev_api_tick_count_;
    prev_api_tick_count_ = dw_current_time;
    /* if (prev_api_tick_count_ == 0) {
      dw_delta_time = 0;
    } */
  }
  /* if (dw_delta_time > 0x0fffffff) {
    // Either wrap-around or data race.
    dw_delta_time = 0;
  }
  if (dw_delta_time > 99999) {
    dw_delta_time = 99999;
  } */
  // tm
  SYSTEMTIME system_time;
  GetSystemTime(&system_time);
  // combine
  sprintf(trace_message, "(%2u:%2u:%2u:%3u |%5lu) ", system_time.wHour,
          system_time.wMinute, system_time.wSecond, system_time.wMilliseconds, 
          static_cast<long>(dw_delta_time));
  return 22;
}

int TraceImpl::AddDateTimeInfo(char* trace_message) const {
  // tick
  /* struct timespec ts;
  timespec_get_systime(&ts);
  prev_api_tick_count_ = timespec_to_millisec(&ts);
  prev_tick_count_ = prev_api_tick_count_; */
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
  sprintf(trace_message, "Local Date: %ls Local Time: %ls", sz_date_str,
          sz_time_str);
  // Include NULL termination (hence + 1).
  return static_cast<int>(strlen(trace_message) + 1);
}

#else // WEBRTC_WIN

int TraceImpl::AddTime(char* trace_message, const TraceLevel level) const {
  // tick
  struct timespec ts;
  timespec_get_systime(&ts);
  int64_t dw_current_time = timespec_to_millisec(&ts);
  int64_t dw_delta_time;
  if (level == kTraceApiCall) {
    dw_delta_time = dw_current_time - prev_tick_count_;
    prev_tick_count_ = dw_current_time;
  } else {
    dw_delta_time = dw_current_time - prev_api_tick_count_;
    prev_api_tick_count_ = dw_current_time;
  }
  // tm
  struct tm buffer;
  const struct tm* system_time = localtime_r(&ts.tv_sec, &buffer);
  const int64_t ms_time = ts.tv_nsec / rtc::kNumNanosecsPerMillisec;
  // combine
  sprintf(trace_message, "(%2u:%2u:%2u:%3lu |%5lu) ", system_time->tm_hour,
          system_time->tm_min, system_time->tm_sec, 
          static_cast<long>(ms_time), static_cast<long>(dw_delta_time));
  // Messages are 22 characters.
  return 22;
}

int TraceImpl::AddDateTimeInfo(char* trace_message) const {
  // datetime
  time_t t;
  time(&t);
  char buffer[26];  // man ctime says buffer should have room for >=26 bytes.
  sprintf(trace_message, "Local Date: %s", ctime_r(&t, buffer));
  int len = static_cast<int>(strlen(trace_message));
  if ('\n' == trace_message[len - 1]) {
    trace_message[len - 1] = '\0';
    --len;
  }
  // Messages is 12 characters.
  return len + 1;
}

#endif // WEBRTC_WIN

int TraceImpl::AddLevel(char* sz_message, const TraceLevel level) const {
  const int kMessageLength = 12;
  switch (level) {
    case kTraceTerseInfo:
      // Add the appropriate amount of whitespace.
      memset(sz_message, ' ', kMessageLength);
      sz_message[kMessageLength] = '\0';
      break;
    case kTraceStateInfo:
      sprintf(sz_message, "STATEINFO ; ");
      break;
    case kTraceWarning:
      sprintf(sz_message, "WARNING   ; ");
      break;
    case kTraceError:
      sprintf(sz_message, "ERROR     ; ");
      break;
    case kTraceCritical:
      sprintf(sz_message, "CRITICAL  ; ");
      break;
    case kTraceInfo:
      sprintf(sz_message, "DEBUGINFO ; ");
      break;
    case kTraceModuleCall:
      sprintf(sz_message, "MODULECALL; ");
      break;
    case kTraceMemory:
      sprintf(sz_message, "MEMORY    ; ");
      break;
    case kTraceTimer:
      sprintf(sz_message, "TIMER     ; ");
      break;
    case kTraceStream:
      sprintf(sz_message, "STREAM    ; ");
      break;
    case kTraceApiCall:
      sprintf(sz_message, "APICALL   ; ");
      break;
    case kTraceDebug:
      sprintf(sz_message, "DEBUG     ; ");
      break;
    default:
      assert(false);
      return 0;
  }
  // All messages are 12 characters.
  return kMessageLength;
}

int TraceImpl::AddModuleAndId(char* trace_message,
                                  const TraceModule module,
                                  const int id) const {
  // Use long int to prevent problems with different definitions of
  // int.
  // TODO(hellner): is this actually a problem? If so, it should be better to
  //                clean up int
  const long int idl = id;
  const int kMessageLength = 25;
  if (idl != -1) {
    const unsigned long int id_engine = id >> 16;
    const unsigned long int id_channel = id & 0xffff;

    switch (module) {
      case kTraceUndefined:
        // Add the appropriate amount of whitespace.
        memset(trace_message, ' ', kMessageLength);
        trace_message[kMessageLength] = '\0';
        break;
      case kTraceVoice:
        sprintf(trace_message, "       VOICE:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceVideo:
        sprintf(trace_message, "       VIDEO:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceUtility:
        sprintf(trace_message, "     UTILITY:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceRtpRtcp:
        sprintf(trace_message, "    RTP/RTCP:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceTransport:
        sprintf(trace_message, "   TRANSPORT:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceAudioCoding:
        sprintf(trace_message, "AUDIO CODING:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceSrtp:
        sprintf(trace_message, "        SRTP:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceAudioMixerServer:
        sprintf(trace_message, " AUDIO MIX/S:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceAudioMixerClient:
        sprintf(trace_message, " AUDIO MIX/C:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceVideoCoding:
        sprintf(trace_message, "VIDEO CODING:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceVideoMixer:
        // Print sleep time and API call
        sprintf(trace_message, "   VIDEO MIX:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceFile:
        sprintf(trace_message, "        FILE:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceAudioProcessing:
        sprintf(trace_message, "  AUDIO PROC:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceAudioDevice:
        sprintf(trace_message, "AUDIO DEVICE:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceVideoRenderer:
        sprintf(trace_message, "VIDEO RENDER:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceVideoCapture:
        sprintf(trace_message, "VIDEO CAPTUR:%5ld %5ld;", id_engine,
                id_channel);
        break;
      case kTraceRemoteBitrateEstimator:
        sprintf(trace_message, "     BWE RBE:%5ld %5ld;", id_engine,
                id_channel);
        break;
    }
  } else {
    switch (module) {
      case kTraceUndefined:
        // Add the appropriate amount of whitespace.
        memset(trace_message, ' ', kMessageLength);
        trace_message[kMessageLength] = '\0';
        break;
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
      case kTraceAudioCoding:
        sprintf(trace_message, "AUDIO CODING:%11ld;", idl);
        break;
      case kTraceSrtp:
        sprintf(trace_message, "        SRTP:%11ld;", idl);
        break;
      case kTraceAudioMixerServer:
        sprintf(trace_message, " AUDIO MIX/S:%11ld;", idl);
        break;
      case kTraceAudioMixerClient:
        sprintf(trace_message, " AUDIO MIX/C:%11ld;", idl);
        break;
      case kTraceVideoCoding:
        sprintf(trace_message, "VIDEO CODING:%11ld;", idl);
        break;
      case kTraceVideoMixer:
        sprintf(trace_message, "   VIDEO MIX:%11ld;", idl);
        break;
      case kTraceFile:
        sprintf(trace_message, "        FILE:%11ld;", idl);
        break;
      case kTraceAudioProcessing:
        sprintf(trace_message, "  AUDIO PROC:%11ld;", idl);
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
    }
  }
  return kMessageLength;
}

int TraceImpl::AddMessage(
    char* trace_message,
    const char msg[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
    const int written_so_far) const {
  int length = 0;
  if (written_so_far >= WEBRTC_TRACE_MAX_MESSAGE_SIZE) {
    return -1;
  }
  // - 2 to leave room for newline and NULL termination.
#if defined(WEBRTC_WIN)
  length = _snprintf(trace_message,
                     WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2,
                     "%s", msg);
  if (length < 0) {
    length = WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2;
    trace_message[length] = 0;
  }
#else
  length = snprintf(trace_message,
                    WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2,
                    "%s", msg);
  if (length < 0 ||
      length > WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2) {
    length = WEBRTC_TRACE_MAX_MESSAGE_SIZE - written_so_far - 2;
    trace_message[length] = 0;
  }
#endif
  // Length with NULL termination.
  return length + 1;
}

void TraceImpl::AddMessageToList(
    const char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE],
    const int length,
    const TraceLevel level) {
  rtc::CritScope lock(&crit_);
  if (callback_) {
    callback_->Print(level, trace_message, length);
    callback_->WriteToFile(trace_message, length);
  }
}

int TraceImpl::SetTraceCallbackImpl(TraceCallback* callback) {
  rtc::CritScope lock(&crit_);
  callback_.reset(callback);
  return 0;
}

void TraceImpl::AddImpl(const TraceLevel level,
                        const TraceModule module,
                        const int id,
                        const char msg[WEBRTC_TRACE_MAX_MESSAGE_SIZE]) {
  if (!TraceCheck(level))
    return;

  char trace_message[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
  char* message_ptr = &trace_message[0];
  int len = AddLevel(message_ptr, level);
  if (len == -1)
    return;

  message_ptr += len;
  int ack_len = len;

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

  len = AddMessage(message_ptr, msg, ack_len);
  if (len == -1)
    return;

  ack_len += len;
  AddMessageToList(trace_message, ack_len, level);
}

// static
TraceImpl* TraceImpl::Instance() {
  static TraceImpl* const trace_impl = new TraceImpl();
  return trace_impl;
  // return StaticInstance(kAddRefNoCreate, level);
}

bool TraceImpl::TraceCheck(const TraceLevel level) const {
  return (level & level_filter()) ? true : false;
}

// static
void Trace::CreateTrace() {
  // TraceImpl::StaticInstance(kAddRef);
}

// static
void Trace::ReturnTrace() {
  // TraceImpl::StaticInstance(kRelease);
}

// static
void Trace::set_level_filter(int filter) {
  rtc::AtomicOps::ReleaseStore(&level_filter_, filter);
}

// static
int Trace::level_filter() {
  return rtc::AtomicOps::AcquireLoad(&level_filter_);
}

// static
/* int Trace::SetTraceFile(const char* file_name,
                            const bool add_file_counter) {
  TraceImpl* trace = TraceImpl::Instance();
  if (trace) {
    int ret_val = trace->SetTraceFileImpl(file_name, add_file_counter);
    ReturnTrace();
    return ret_val;
  }
  return -1;
} */

int Trace::SetTraceCallback(TraceCallback* callback) {
  TraceImpl* trace = TraceImpl::Instance();
  if (trace) {
    int ret_val = trace->SetTraceCallbackImpl(callback);
    ReturnTrace();
    return ret_val;
  }
  return -1;
}

void Trace::Add(const TraceLevel level, const TraceModule module,
                const int id, const char* msg, ...) {
  TraceImpl* trace = TraceImpl::Instance();
  if (trace) {
    if (trace->TraceCheck(level)) {
      char temp_buff[WEBRTC_TRACE_MAX_MESSAGE_SIZE];
      char* buff = NULL;
      if (msg) {
        va_list args;
        va_start(args, msg);
#if defined(WEBRTC_WIN)
        _vsnprintf(temp_buff, WEBRTC_TRACE_MAX_MESSAGE_SIZE - 1, msg, args);
#else
        vsnprintf(temp_buff, WEBRTC_TRACE_MAX_MESSAGE_SIZE - 1, msg, args);
#endif // WEBRTC_WIN
        va_end(args);
        buff = temp_buff;
      }
      trace->AddImpl(level, module, id, buff);
    }
    ReturnTrace(); // |StaticInstance| should be called in pairs
  }
}

}  // namespace webrtc
