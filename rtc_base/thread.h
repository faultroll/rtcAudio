/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_THREAD_H_
#define RTC_BASE_THREAD_H_

#include <algorithm>
#include <list>
#include <string>
#include <queue>
#include <vector>

#include "rtc_base/constructor_magic.h"
#include "rtc_base/thread_message.h"
#include "rtc_base/event.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/message_handler.h"
#include "rtc_base/socket_server.h"
#include "rtc_base/thread_annotations.h"

namespace rtc {

class Thread;

class RTC_EXPORT ThreadManager {
 public:
  static const int kForever = -1;

  // Singleton, constructor and destructor are private.
  static ThreadManager* Instance();

  static void Add(Thread* message_queue);
  static void Remove(Thread* message_queue);
  static void Clear(MessageHandler* handler);

  Thread* CurrentThread();
  void SetCurrentThread(Thread* thread);

  // Returns a thread object with its thread_ ivar set
  // to whatever the OS uses to represent the thread.
  // If there already *is* a Thread object corresponding to this thread,
  // this method will return that.  Otherwise it creates a new Thread
  // object whose wrapped() method will return true, and whose
  // handle will, on Win32, be opened with only synchronization privileges -
  // if you need more privilegs, rather than changing this method, please
  // write additional code to adjust the privileges, or call a different
  // factory method of your own devising, because this one gets used in
  // unexpected contexts (like inside browser plugins) and it would be a
  // shame to break it.  It is also conceivable on Win32 that we won't even
  // be able to get synchronization privileges, in which case the result
  // will have a null handle.
  Thread* WrapCurrentThread();
  void UnwrapCurrentThread();

  bool IsMainThread();

 private:
  ThreadManager();
  ~ThreadManager();

  void SetCurrentThreadInternal(Thread* thread);
  void AddInternal(Thread* message_queue);
  void RemoveInternal(Thread* message_queue);
  void ClearInternal(MessageHandler* handler);

  // This list contains all live Threads.
  std::vector<Thread*> message_queues_ RTC_GUARDED_BY(crit_);

  // Methods that don't modify the list of message queues may be called in a
  // re-entrant fashion. "processing_" keeps track of the depth of re-entrant
  // calls.
  CriticalSection crit_;
  size_t processing_ RTC_GUARDED_BY(crit_) = 0;

  Tss key_;

  // The thread to potentially autowrap.
  const Thrd main_thread_ref_;

  RTC_DISALLOW_COPY_AND_ASSIGN(ThreadManager);
};

// WARNING! SUBCLASSES MUST CALL Stop() IN THEIR DESTRUCTORS!  See ~Thread().

class RTC_LOCKABLE RTC_EXPORT Thread {
 public:
  static const int kForever = -1;

  // Create a new Thread and optionally assign it to the passed
  // SocketServer. Subclasses that override Clear should pass false for
  // init_queue and call DoInit() from their constructor to prevent races
  // with the ThreadManager using the object while the vtable is still
  // being created.
  explicit Thread(SocketServer* ss);
  explicit Thread(std::unique_ptr<SocketServer> ss);

  // Constructors meant for subclasses; they should call DoInit themselves and
  // pass false for |do_init|, so that DoInit is called only on the fully
  // instantiated class, which avoids a vptr data race.
  Thread(SocketServer* ss, bool do_init);
  Thread(std::unique_ptr<SocketServer> ss, bool do_init);

  // NOTE: ALL SUBCLASSES OF Thread MUST CALL Stop() IN THEIR DESTRUCTORS (or
  // guarantee Stop() is explicitly called before the subclass is destroyed).
  // This is required to avoid a data race between the destructor modifying the
  // vtable, and the Thread::PreRun calling the virtual method Run().

  // NOTE: SUBCLASSES OF Thread THAT OVERRIDE Clear MUST CALL
  // DoDestroy() IN THEIR DESTRUCTORS! This is required to avoid a data race
  // between the destructor modifying the vtable, and the ThreadManager
  // calling Clear on the object from a different thread.
  virtual ~Thread();

  static std::unique_ptr<Thread> CreateWithSocketServer();
  static std::unique_ptr<Thread> Create();
  static Thread* Current();

  SocketServer* socketserver();

  // Note: The behavior of Thread has changed.  When a thread is stopped,
  // futher Posts and Sends will fail.  However, any pending Sends and *ready*
  // Posts (as opposed to unexpired delayed Posts) will be delivered before
  // Get (or Peek) returns false.  By guaranteeing delivery of those messages,
  // we eliminate the race condition when an MessageHandler and Thread
  // may be destroyed independently of each other.
  virtual void Quit();
  virtual bool IsQuitting();
  virtual void Restart();
  // Not all message queues actually process messages (such as SignalThread).
  // In those cases, it's important to know, before posting, that it won't be
  // Processed.  Normally, this would be true until IsQuitting() is true.
  virtual bool IsProcessingMessages();

  // Get() will process I/O until:
  //  1) A message is available (returns true)
  //  2) cmsWait seconds have elapsed (returns false)
  //  3) Stop() is called (returns false)
  virtual bool Get(Message* pmsg,
                   int cmsWait = kForever,
                   bool process_io = true);
  virtual bool Peek(Message* pmsg, int cmsWait = 0);
  virtual void Post(const Location& posted_from,
                    MessageHandler* phandler,
                    uint32_t id = 0,
                    MessageData* pdata = nullptr);
  virtual void PostDelayed(const Location& posted_from,
                           int delay_ms,
                           MessageHandler* phandler,
                           uint32_t id = 0,
                           MessageData* pdata = nullptr);
  virtual void PostAt(const Location& posted_from,
                      int64_t run_at_ms,
                      MessageHandler* phandler,
                      uint32_t id = 0,
                      MessageData* pdata = nullptr);
  virtual void Clear(MessageHandler* phandler,
                     uint32_t id = MQID_ANY,
                     MessageList* removed = nullptr);
  virtual void Dispatch(Message* pmsg);

  // Amount of time until the next message can be retrieved
  virtual int GetDelay();

  bool empty() const { return size() == 0u; }
  size_t size() const {
    CritScope cs(&crit_);
    return messages_.size() + delayed_messages_.size() + (fPeekKeep_ ? 1u : 0u);
  }

  // Internally posts a message which causes the doomed object to be deleted
  template <class T>
  void Dispose(T* doomed) {
    if (doomed) {
      Post(RTC_FROM_HERE, nullptr, MQID_DISPOSE, new DisposeData<T>(doomed));
    }
  }

  bool IsCurrent() const;

  // Sleeps the thread for the specified number of milliseconds, during
  // which time no processing is performed. Returns false if sleeping was
  // interrupted by a signal (POSIX only).
  /* static */ bool SleepMs(int millis);

  // Sets the thread's name, for debugging. Must be called before Start().
  // If |obj| is non-null, its value is appended to |name|.
  const std::string& name() const { return name_; }
  bool SetName(const std::string& name, const void* obj = nullptr);

  // Sets the expected processing time in ms. The thread will write
  // log messages when Invoke() takes more time than this.
  // Default is 50 ms.
  void SetDispatchWarningMs(int deadline);

  // Starts the execution of the thread.
  bool Start();

  // Tells the thread to stop and waits until it is joined.
  // Never call Stop on the current thread.  Instead use the inherited Quit
  // function which will exit the base Thread without terminating the
  // underlying OS thread.
  virtual void Stop();

  // By default, Thread::Run() calls ProcessMessages(kForever).  To do other
  // work, override Run().  To receive and dispatch messages, call
  // ProcessMessages occasionally.
  virtual void Run();

  virtual void Send(const Location& posted_from,
                    MessageHandler* phandler,
                    uint32_t id = 0,
                    MessageData* pdata = nullptr);

  // ProcessMessages will process I/O and dispatch messages until:
  //  1) cms milliseconds have elapsed (returns true)
  //  2) Stop() is called (returns false)
  bool ProcessMessages(int cms);

  // Returns true if this is a thread that we created using the standard
  // constructor, false if it was created by a call to
  // ThreadManager::WrapCurrentThread().  The main thread of an application
  // is generally not owned, since the OS representation of the thread
  // obviously exists before we can get to it.
  // You cannot call Start on non-owned threads.
  bool IsOwned();

  // These functions are public to avoid injecting test hooks. Don't call them
  // outside of tests.
  // This method should be called when thread is created using non standard
  // method, like derived implementation of rtc::Thread and it can not be
  // started by calling Start(). This will set started flag to true and
  // owned to false. This must be called from the current thread.
  bool WrapCurrent();
  void UnwrapCurrent();

 protected:
  // DelayedMessage goes into a priority queue, sorted by trigger time. Messages
  // with the same trigger time are processed in num_ (FIFO) order.
  class DelayedMessage {
   public:
    DelayedMessage(int64_t delay,
                   int64_t run_time_ms,
                   uint32_t num,
                   const Message& msg)
        : delay_ms_(delay),
          run_time_ms_(run_time_ms),
          message_number_(num),
          msg_(msg) {}

    bool operator<(const DelayedMessage& dmsg) const {
      return (dmsg.run_time_ms_ < run_time_ms_) ||
             ((dmsg.run_time_ms_ == run_time_ms_) &&
              (dmsg.message_number_ < message_number_));
    }

    int64_t delay_ms_;  // for debugging
    int64_t run_time_ms_;
    // Monotonicaly incrementing number used for ordering of messages
    // targeted to execute at the same time.
    uint32_t message_number_;
    Message msg_;
  };

  class PriorityQueue : public std::priority_queue<DelayedMessage> {
   public:
    container_type& container() { return c; }
    void reheap() { make_heap(c.begin(), c.end(), comp); }
  };

  void DoDelayPost(const Location& posted_from,
                   int64_t cmsDelay,
                   int64_t tstamp,
                   MessageHandler* phandler,
                   uint32_t id,
                   MessageData* pdata);

  // Perform initialization, subclasses must call this from their constructor
  // if false was passed as init_queue to the Thread constructor.
  void DoInit();

  // Does not take any lock. Must be called either while holding crit_, or by
  // the destructor (by definition, the latter has exclusive access).
  void ClearInternal(MessageHandler* phandler,
                     uint32_t id,
                     MessageList* removed) RTC_EXCLUSIVE_LOCKS_REQUIRED(&crit_);

  // Perform cleanup; subclasses must call this from the destructor,
  // and are not expected to actually hold the lock.
  void DoDestroy() RTC_EXCLUSIVE_LOCKS_REQUIRED(&crit_);

  void WakeUpSocketServer();

  // Same as WrapCurrent except that it never fails as it does not try to
  // acquire the synchronization access of the thread. The caller should never
  // call Stop() or Join() on this thread.
  void SafeWrapCurrent();

  // Blocks the calling thread until this thread has terminated.
  void Join();

 private:
  static const int kSlowDispatchLoggingThreshold = 50;  // 50 ms

/* #if defined(WEBRTC_WIN)
  static DWORD WINAPI PreRun(LPVOID context);
#else
  static void* PreRun(void* pv);
#endif */
  static void PreRun(void* pv);

  // ThreadManager calls this instead WrapCurrent() because
  // ThreadManager::Instance() cannot be used while ThreadManager is
  // being created.
  // The method tries to get synchronization rights of the thread on Windows if
  // |need_synchronize_access| is true.
  bool WrapCurrentWithThreadManager(ThreadManager* thread_manager,
                                    bool need_synchronize_access);

  // Return true if the thread is currently running.
  bool IsRunning();

#if 0 /* defined(USING_SENDLIST) */
  struct _SendMessage {
    _SendMessage() {}
    Thread *thread;
    Message msg;
    bool *ready;
  };

  void ReceiveSends();

  // Processes received "Send" requests. If |source| is not null, only requests
  // from |source| are processed, otherwise, all requests are processed.
  void ReceiveSendsFromThread(const Thread* source);

  // If |source| is not null, pops the first "Send" message from |source| in
  // |sendlist_|, otherwise, pops the first "Send" message of |sendlist_|.
  // The caller must lock |crit_| before calling.
  // Returns true if there is such a message.
  bool PopSendMessageFromThread(const Thread* source, _SendMessage* msg);

  void InvokeInternal(const Location& posted_from, MessageHandler* handler);

  std::list<_SendMessage> sendlist_;

#else
  class _SendMessage final : public rtc::MessageData {
   public:
    Thread *thread;
    Message msg;
    bool *ready;
    Event *done;
  };

  /* class _SetDispatchWarningMsMessage final : public rtc::MessageData {
   public:
    Thread *thread;
    int deadline;
  }; */

  class _SleepMsMessage final : public rtc::MessageData {
   public:
    Thread *thread;
    int milliseconds;
  };

  class QueuedTaskHandler final : public MessageHandler {
   public:
    enum Operation {
      kSend,
      /* kSetDispatchWarningMs, */
      kSleepMs,
    };
    QueuedTaskHandler() {}
    void OnMessage(Message* msg) override;
  };

  // Runs webrtc::QueuedTask posted to the Thread.
  QueuedTaskHandler queued_task_handler_;

#endif

  bool fPeekKeep_;
  Message msgPeek_;
  MessageList messages_ RTC_GUARDED_BY(crit_);
  PriorityQueue delayed_messages_ RTC_GUARDED_BY(crit_);
  uint32_t delayed_next_num_ RTC_GUARDED_BY(crit_);
  CriticalSection crit_;
  bool fInitialized_;
  bool fDestroyed_;
  volatile int stop_;

  // The SocketServer might not be owned by Thread.
  SocketServer* const ss_;
  // Used if SocketServer ownership lies with |this|.
  std::unique_ptr<SocketServer> own_ss_;

  std::string name_;

  // TODO(tommi): Add thread checks for proper use of control methods.
  // Ideally we should be able to just use PlatformThread.

/* #if defined(WEBRTC_POSIX)
  pthread_t thread_ = 0;
#endif

#if defined(WEBRTC_WIN)
  HANDLE thread_ = nullptr;
  DWORD thread_id_ = 0;
#endif */
  PlatformThread* thread_;
  Thrd thread_ref_;

  // Indicates whether or not ownership of the worker thread lies with
  // this instance or not. (i.e. owned_ == !wrapped).
  // Must only be modified when the worker thread is not running.
  bool owned_ = true;

  /* // Only touched from the worker thread itself.
  bool blocking_calls_allowed_ = true; */

  friend class ThreadManager;

  int dispatch_warning_ms_ RTC_GUARDED_BY(this) = kSlowDispatchLoggingThreshold;

  RTC_DISALLOW_COPY_AND_ASSIGN(Thread);
};

#if 0 /* defined(USING_SENDLIST) */
// AutoThread automatically installs itself at construction
// uninstalls at destruction, if a Thread object is
// _not already_ associated with the current OS thread.
//
// NOTE: *** This class should only be used by tests ***
//
class AutoThread : public Thread {
 public:
  AutoThread();
  ~AutoThread();

 private:
  RTC_DISALLOW_COPY_AND_ASSIGN(AutoThread);
};

// AutoSocketServerThread automatically installs itself at
// construction and uninstalls at destruction. If a Thread object is
// already associated with the current OS thread, it is temporarily
// disassociated and restored by the destructor.

class AutoSocketServerThread : public Thread {
 public:
  explicit AutoSocketServerThread(SocketServer* ss);
  ~AutoSocketServerThread();

 private:
  rtc::Thread* old_thread_;

  RTC_DISALLOW_COPY_AND_ASSIGN(AutoSocketServerThread);
};

#endif

}  // namespace rtc

#endif  // RTC_BASE_THREAD_H_
