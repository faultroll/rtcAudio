/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef RTC_BASE_THREAD_MESSAGE_H_
#define RTC_BASE_THREAD_MESSAGE_H_

#include <list>

#include "rtc_base/location.h"
#include "rtc_base/messagehandler.h"

namespace rtc {

// Derive from this for specialized data
// App manages lifetime, except when messages are purged

class MessageData {
 public:
  MessageData() {}
  virtual ~MessageData() {}
};

template <class T>
class DisposeData : public MessageData {
 public:
  explicit DisposeData(T* data) : data_(data) {}
  virtual ~DisposeData() { delete data_; }

 private:
  T* data_;
};

const uint32_t MQID_ANY = static_cast<uint32_t>(-1);
const uint32_t MQID_DISPOSE = static_cast<uint32_t>(-2);

// No destructor

struct Message {
  Message() : phandler(nullptr), message_id(0), pdata(nullptr) {}
  inline bool Match(MessageHandler* handler, uint32_t id) const {
    return (handler == nullptr || handler == phandler) &&
           (id == MQID_ANY || id == message_id);
  }
  Location posted_from;
  MessageHandler* phandler;
  uint32_t message_id;
  MessageData* pdata;
};

typedef std::list<Message> MessageList;
}  // namespace rtc
#endif  // RTC_BASE_THREAD_MESSAGE_H_
