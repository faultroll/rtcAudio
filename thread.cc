/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_base/thread.h"

#include "rtc_base/checks.h"
#include "rtc_base/null_socket_server.h"
#include "rtc_base/platform_thread.h"
#include "rtc_base/time_utils.h"
#include "rtc_base/system/compile_magic.h"  // NOLINT(build/include)

namespace rtc {

namespace {

class MessageHandlerWithTask final : public MessageHandler {
 public:
  MessageHandlerWithTask() = default;

  void OnMessage(Message* msg) override {
    static_cast<rtc_thread_internal::MessageLikeTask*>(msg->pdata)->Run();
    delete msg->pdata;
  }

 private:
  ~MessageHandlerWithTask() override {}

  RTC_DISALLOW_COPY_AND_ASSIGN(MessageHandlerWithTask);
};

class RTC_SCOPED_LOCKABLE MarkProcessingCritScope {
 public:
  MarkProcessingCritScope(const CriticalSection* cs, size_t* processing)
      RTC_EXCLUSIVE_LOCK_FUNCTION(cs)
      : cs_(cs), processing_(processing) {
    cs_->Enter();
    *processing_ += 1;
  }

  ~MarkProcessingCritScope() RTC_UNLOCK_FUNCTION() {
    *processing_ -= 1;
    cs_->Leave();
  }

 private:
  const CriticalSection* const cs_;
  size_t* processing_;

  RTC_DISALLOW_COPY_AND_ASSIGN(MarkProcessingCritScope);
};

}  // namespace

ThreadManager* ThreadManager::Instance() {
  // if we use |static ThreadManager thread_manager;| here, 
  // memleak(message_queues_&key_&thread_manager) will be fixed, 
  // but seems its not the usual singleton, so just let it unchanged
  static ThreadManager* const thread_manager = new ThreadManager();
  return thread_manager;
}

ThreadManager::~ThreadManager() {
  // By above RTC_DEFINE_STATIC_LOCAL.
  TssDelete(key_);
  RTC_NOTREACHED() /* << "ThreadManager should never be destructed." */;
}

// static
void ThreadManager::Add(Thread* message_queue) {
  return Instance()->AddInternal(message_queue);
}
void ThreadManager::AddInternal(Thread* message_queue) {
  CritScope cs(&crit_);
  // Prevent changes while the list of message queues is processed.
  RTC_DCHECK_EQ(processing_, 0);
  message_queues_.push_back(message_queue);
}

// static
void ThreadManager::Remove(Thread* message_queue) {
  return Instance()->RemoveInternal(message_queue);
}
void ThreadManager::RemoveInternal(Thread* message_queue) {
  {
    CritScope cs(&crit_);
    // Prevent changes while the list of message queues is processed.
    RTC_DCHECK_EQ(processing_, 0);
    std::vector<Thread*>::iterator iter;
    iter = std::find(message_queues_.begin(), message_queues_.end(),
                     message_queue);
    if (iter != message_queues_.end()) {
      message_queues_.erase(iter);
    }
#if RTC_DCHECK_IS_ON
    RemoveFromSendGraph(message_queue);
#endif
  }
}

#if RTC_DCHECK_IS_ON
void ThreadManager::RemoveFromSendGraph(Thread* thread) {
  for (auto it = send_graph_.begin(); it != send_graph_.end();) {
    if (it->first == thread) {
      it = send_graph_.erase(it);
    } else {
      it->second.erase(thread);
      ++it;
    }
  }
}

void ThreadManager::RegisterSendAndCheckForCycles(Thread* source,
                                                  Thread* target) {
  CritScope cs(&crit_);
  std::deque<Thread*> all_targets({target});
  // We check the pre-existing who-sends-to-who graph for any path from target
  // to source. This loop is guaranteed to terminate because per the send graph
  // invariant, there are no cycles in the graph.
  for (size_t i = 0; i < all_targets.size(); i++) {
    const auto& targets = send_graph_[all_targets[i]];
    all_targets.insert(all_targets.end(), targets.begin(), targets.end());
  }
  /* RTC_CHECK_EQ(absl::c_count(all_targets, source), 0)
      << " send loop between " << source->name() << " and " << target->name(); */
  RTC_CHECK_EQ(std::count(all_targets.begin(), all_targets.end(), source), 0);

  // We may now insert source -> target without creating a cycle, since there
  // was no path from target to source per the prior CHECK.
  send_graph_[source].insert(target);
}
#endif

// static
void ThreadManager::Clear(MessageHandler* handler) {
  return Instance()->ClearInternal(handler);
}
void ThreadManager::ClearInternal(MessageHandler* handler) {
  // Deleted objects may cause re-entrant calls to ClearInternal. This is
  // allowed as the list of message queues does not change while queues are
  // cleared.
  MarkProcessingCritScope cs(&crit_, &processing_);
  for (Thread* queue : message_queues_) {
    queue->Clear(handler);
  }
}

// static
Thread* Thread::Current() {
  ThreadManager* manager = ThreadManager::Instance();
  Thread* thread = manager->CurrentThread();

#if 1 /* !defined(NO_MAIN_THREAD_WRAPPING) */
  // Only autowrap the thread which instantiated the ThreadManager.
  if (!thread && manager->IsMainThread()) {
    thread = new Thread(SocketServer::CreateDefault());
    thread->WrapCurrentWithThreadManager(manager, true);
  }
#endif

  return thread;
}

ThreadManager::ThreadManager()
    : key_(TssCreate()), main_thread_ref_(ThrdCurrent()) {}

Thread* ThreadManager::CurrentThread() {
  return static_cast<Thread*>(TssGet(key_));
}

void ThreadManager::SetCurrentThreadInternal(Thread* thread) {
  TssSet(key_, thread);
}

void ThreadManager::SetCurrentThread(Thread* thread) {
#if 0 // RTC_DCHECK_IS_ON // TODO logs using checks (FatalMessage is Logging with abort)
  if (CurrentThread() && thread) {
    /* RTC_DLOG(LS_ERROR) << "SetCurrentThread: Overwriting an existing value?"; */
  }
#endif  // RTC_DCHECK_IS_ON

  /* if (thread) {
    thread->EnsureIsCurrentTaskQueue();
  } else {
    Thread* current = CurrentThread();
    if (current) {
      // The current thread is being cleared, e.g. as a result of
      // UnwrapCurrent() being called or when a thread is being stopped
      // (see PreRun()). This signals that the Thread instance is being detached
      // from the thread, which also means that TaskQueue::Current() must not
      // return a pointer to the Thread instance.
      current->ClearCurrentTaskQueue();
    }
  } */

  SetCurrentThreadInternal(thread);
}

Thread* ThreadManager::WrapCurrentThread() {
  Thread* result = CurrentThread();
  if (nullptr == result) {
    result = new Thread(SocketServer::CreateDefault());
    result->WrapCurrentWithThreadManager(this, true);
  }
  return result;
}

void ThreadManager::UnwrapCurrentThread() {
  Thread* t = CurrentThread();
  if (t && !(t->IsOwned())) {
    t->UnwrapCurrent();
    delete t;
  }
}

bool ThreadManager::IsMainThread() {
  return ThrdEqual(ThrdCurrent(), main_thread_ref_);
}

Thread::Thread(SocketServer* ss) : Thread(ss, /*do_init=*/true) {}

Thread::Thread(std::unique_ptr<SocketServer> ss)
    : Thread(std::move(ss), /*do_init=*/true) {}

Thread::Thread(SocketServer* ss, bool do_init)
    : fPeekKeep_(false),
      delayed_next_num_(0),
      fInitialized_(false),
      fDestroyed_(false),
      stop_(0),
      ss_(ss) {
  RTC_DCHECK(ss);
  ss_->SetMessageQueue(this);
  /* SetName("Thread", this);  // default name */
  if (do_init) {
    DoInit();
  }
}

Thread::Thread(std::unique_ptr<SocketServer> ss, bool do_init)
    : Thread(ss.get(), do_init) {
  own_ss_ = std::move(ss);
}

Thread::~Thread() {
  Stop();
  DoDestroy();
}

void Thread::DoInit() {
  if (fInitialized_) {
    return;
  }

  fInitialized_ = true;
  thread_ = new PlatformThread(PreRun, this);
  thread_ref_ = kNullThrd;
  ThreadManager::Add(this);
}

void Thread::DoDestroy() {
  if (fDestroyed_) {
    return;
  }

  fDestroyed_ = true;
  if (thread_ != nullptr) {
    delete thread_;
  }
  // The signal is done from here to ensure
  // that it always gets called when the queue
  // is going away.
  if (ss_) {
    ss_->SetMessageQueue(nullptr);
  }
  ThreadManager::Remove(this);
  ClearInternal(nullptr, MQID_ANY, nullptr);
}

SocketServer* Thread::socketserver() {
  return ss_;
}

void Thread::WakeUpSocketServer() {
  ss_->WakeUp();
}

void Thread::Quit() {
  AtomicOps::ReleaseStore(&stop_, 1);
  WakeUpSocketServer();
}

bool Thread::IsQuitting() {
  return AtomicOps::AcquireLoad(&stop_) != 0;
}

void Thread::Restart() {
  AtomicOps::ReleaseStore(&stop_, 0);
}

bool Thread::Peek(Message* pmsg, int cmsWait) {
  if (fPeekKeep_) {
    *pmsg = msgPeek_;
    return true;
  }
  if (!Get(pmsg, cmsWait))
    return false;
  msgPeek_ = *pmsg;
  fPeekKeep_ = true;
  return true;
}

bool Thread::Get(Message* pmsg, int cmsWait, bool process_io) {
  // Return and clear peek if present
  // Always return the peek if it exists so there is Peek/Get symmetry

  if (fPeekKeep_) {
    *pmsg = msgPeek_;
    fPeekKeep_ = false;
    return true;
  }

  // Get w/wait + timer scan / dispatch + socket / event multiplexer dispatch

  int64_t cmsTotal = cmsWait;
  int64_t cmsElapsed = 0;
  int64_t msStart = TimeMillis();
  int64_t msCurrent = msStart;
  while (true) {
    // Check for posted events
    int64_t cmsDelayNext = kForever;
    bool first_pass = true;
    while (true) {
      // All queue operations need to be locked, but nothing else in this loop
      // (specifically handling disposed message) can happen inside the crit.
      // Otherwise, disposed MessageHandlers will cause deadlocks.
      {
        CritScope cs(&crit_);
        // On the first pass, check for delayed messages that have been
        // triggered and calculate the next trigger time.
        if (first_pass) {
          first_pass = false;
          while (!delayed_messages_.empty()) {
            if (msCurrent < delayed_messages_.top().run_time_ms_) {
              cmsDelayNext =
                  TimeDiff(delayed_messages_.top().run_time_ms_, msCurrent);
              break;
            }
            messages_.push_back(delayed_messages_.top().msg_);
            delayed_messages_.pop();
          }
        }
        // Pull a message off the message queue, if available.
        if (messages_.empty()) {
          break;
        } else {
          *pmsg = messages_.front();
          messages_.pop_front();
        }
      }  // crit_ is released here.

      // If this was a dispose message, delete it and skip it.
      if (MQID_DISPOSE == pmsg->message_id) {
        RTC_DCHECK(nullptr == pmsg->phandler);
        delete pmsg->pdata;
        *pmsg = Message();
        continue;
      }
      return true;
    }

    if (IsQuitting())
      break;

    // Which is shorter, the delay wait or the asked wait?

    int64_t cmsNext;
    if (cmsWait == kForever) {
      cmsNext = cmsDelayNext;
    } else {
      cmsNext = std::max<int64_t>(0, cmsTotal - cmsElapsed);
      if ((cmsDelayNext != kForever) && (cmsDelayNext < cmsNext))
        cmsNext = cmsDelayNext;
    }

    {
      // Wait and multiplex in the meantime
      if (!ss_->Wait(static_cast<int>(cmsNext), process_io))
        return false;
    }

    // If the specified timeout expired, return

    msCurrent = TimeMillis();
    cmsElapsed = TimeDiff(msCurrent, msStart);
    if (cmsWait != kForever) {
      if (cmsElapsed >= cmsWait)
        return false;
    }
  }
  return false;
}

void Thread::Post(const Location& posted_from,
                  MessageHandler* phandler,
                  uint32_t id,
                  MessageData* pdata) {
  if (IsQuitting()) {
    delete pdata;
    return;
  }

  // Keep thread safe
  // Add the message to the end of the queue
  // Signal for the multiplexer to return

  {
    CritScope cs(&crit_);
    Message msg;
    msg.posted_from = posted_from;
    msg.phandler = phandler;
    msg.message_id = id;
    msg.pdata = pdata;
    messages_.push_back(msg);
  }
  WakeUpSocketServer();
}

void Thread::PostDelayed(const Location& posted_from,
                         int delay_ms,
                         MessageHandler* phandler,
                         uint32_t id,
                         MessageData* pdata) {
  return DoDelayPost(posted_from, delay_ms, TimeAfter(delay_ms), phandler, id,
                     pdata);
}

void Thread::PostAt(const Location& posted_from,
                    int64_t run_at_ms,
                    MessageHandler* phandler,
                    uint32_t id,
                    MessageData* pdata) {
  return DoDelayPost(posted_from, TimeUntil(run_at_ms), run_at_ms, phandler, id,
                     pdata);
}

void Thread::DoDelayPost(const Location& posted_from,
                         int64_t delay_ms,
                         int64_t run_at_ms,
                         MessageHandler* phandler,
                         uint32_t id,
                         MessageData* pdata) {
  if (IsQuitting()) {
    delete pdata;
    return;
  }

  // Keep thread safe
  // Add to the priority queue. Gets sorted soonest first.
  // Signal for the multiplexer to return.

  {
    CritScope cs(&crit_);
    Message msg;
    msg.posted_from = posted_from;
    msg.phandler = phandler;
    msg.message_id = id;
    msg.pdata = pdata;
    DelayedMessage delayed(delay_ms, run_at_ms, delayed_next_num_, msg);
    delayed_messages_.push(delayed);
    // If this message queue processes 1 message every millisecond for 50 days,
    // we will wrap this number.  Even then, only messages with identical times
    // will be misordered, and then only briefly.  This is probably ok.
    ++delayed_next_num_;
    RTC_DCHECK_NE(0, delayed_next_num_);
  }
  WakeUpSocketServer();
}

int Thread::GetDelay() {
  CritScope cs(&crit_);

  if (!messages_.empty())
    return 0;

  if (!delayed_messages_.empty()) {
    int delay = TimeUntil(delayed_messages_.top().run_time_ms_);
    if (delay < 0)
      delay = 0;
    return delay;
  }

  return kForever;
}

void Thread::ClearInternal(MessageHandler* phandler,
                           uint32_t id,
                           MessageList* removed) {
  // Remove messages with phandler

  if (fPeekKeep_ && msgPeek_.Match(phandler, id)) {
    if (removed) {
      removed->push_back(msgPeek_);
    } else {
      delete msgPeek_.pdata;
    }
    fPeekKeep_ = false;
  }

  // Remove from ordered message queue

  for (MessageList::iterator it = messages_.begin(); it != messages_.end();) {
    if (it->Match(phandler, id)) {
      if (removed) {
        removed->push_back(*it);
      } else {
        delete it->pdata;
      }
      it = messages_.erase(it);
    } else {
      ++it;
    }
  }

  // Remove from priority queue. Not directly iterable, so use this approach

  PriorityQueue::container_type::iterator new_end = delayed_messages_.container().begin();
  for (PriorityQueue::container_type::iterator it = new_end;
       it != delayed_messages_.container().end(); ++it) {
    if (it->msg_.Match(phandler, id)) {
      if (removed) {
        removed->push_back(it->msg_);
      } else {
        delete it->msg_.pdata;
      }
    } else {
      *new_end++ = *it;
    }
  }
  delayed_messages_.container().erase(new_end,
                                      delayed_messages_.container().end());
  delayed_messages_.reheap();
}

void Thread::Dispatch(Message* pmsg) {
  int64_t start_time = TimeMillis();
  pmsg->phandler->OnMessage(pmsg);
  int64_t end_time = TimeMillis();
  int64_t diff = TimeDiff(end_time, start_time);
  if (diff >= dispatch_warning_ms_) {
    /* RTC_LOG(LS_INFO) << "Message to " << name() << " took " << diff
                     << "ms to dispatch. Posted from: "
                     << pmsg->posted_from.ToString(); */
    // To avoid log spew, move the warning limit to only give warning
    // for delays that are larger than the one observed.
    dispatch_warning_ms_ = diff + 1;
  }
}

bool Thread::IsCurrent() const {
  return ThreadManager::Instance()->CurrentThread() == this;
}

std::unique_ptr<Thread> Thread::CreateWithSocketServer() {
  return std::unique_ptr<Thread>(new Thread(SocketServer::CreateDefault()));
}

std::unique_ptr<Thread> Thread::Create() {
  return std::unique_ptr<Thread>(
      new Thread(std::unique_ptr<SocketServer>(new NullSocketServer())));
}

bool Thread::SleepMs(int milliseconds) {
#if 0 // using funcs in platform_thread_types
#if defined(WEBRTC_WIN)
  ::Sleep(milliseconds);
  return true;
#else
  // POSIX has both a usleep() and a nanosleep(), but the former is deprecated,
  // so we use nanosleep() even though it has greater precision than necessary.
  struct timespec ts;
  ts.tv_sec = milliseconds / 1000;
  ts.tv_nsec = (milliseconds % 1000) * 1000000;
  int ret = nanosleep(&ts, nullptr);
  if (ret != 0) {
    /* RTC_LOG_ERR(LS_WARNING) << "nanosleep() returning early"; */
    return false;
  }
  return true;
#endif
#else // 0
  if (!IsCurrent()) {
    _SleepMsMessage *smsg = new _SleepMsMessage;
    smsg->thread = this;
    smsg->milliseconds = milliseconds;
    Post(RTC_FROM_HERE, &queued_task_handler_, QueuedTaskHandler::kSleepMs, smsg);
    return true;
  }
  return ThrdSleep(milliseconds);
#endif // 0
}

bool Thread::SetName(const std::string& name, const void* obj) {
  /* RTC_DCHECK(!IsRunning());

  name_ = name;
  if (obj) {
    // The %p specifier typically produce at most 16 hex digits, possibly with a
    // 0x prefix. But format is implementation defined, so add some margin.
    char buf[30];
    snprintf(buf, sizeof(buf), " 0x%p", obj);
    name_ += buf;
  }
  return true; */
  return thread_->SetName(name, obj);
}

void Thread::SetDispatchWarningMs(int deadline) {
#if 0 // No need to check whether if it's |current_thread| here
  if (!IsCurrent()) {
#if 0 /* defined(USING_SENDLIST) */
    // what to do here?
#else
    /* PostTask(webrtc::ToQueuedTask(
        [this, deadline]() { SetDispatchWarningMs(deadline); })); */
    _SetDispatchWarningMsMessage *smsg = new _SetDispatchWarningMsMessage;
    smsg->thread = this;
    smsg->deadline = deadline;
    Post(RTC_FROM_HERE, &queued_task_handler_, QueuedTaskHandler::kSetDispatchWarningMs, smsg);
#endif
    return;
  }
#endif // 0
  dispatch_warning_ms_ = deadline;
}

bool Thread::Start() {
  RTC_DCHECK(!IsRunning());

  if (IsRunning())
    return false;

  Restart();  // reset IsQuitting() if the thread is being restarted

  // Make sure that ThreadManager is created on the main thread before
  // we start a new thread.
  ThreadManager::Instance();

  owned_ = true;

/* #if defined(WEBRTC_WIN)
  thread_ = CreateThread(nullptr, 0, PreRun, this, 0, &thread_id_);
  if (!thread_) {
    return false;
  }
#elif defined(WEBRTC_POSIX)
  pthread_attr_t attr;
  pthread_attr_init(&attr);

  int error_code = pthread_create(&thread_, &attr, PreRun, this);
  if (0 != error_code) {
    // RTC_LOG(LS_ERROR) << "Unable to create pthread, error " << error_code;
    thread_ = 0;
    return false;
  }
  RTC_DCHECK(thread_);
#endif */
  thread_->Start();
  thread_ref_ = thread_->GetThreadRef();
  return true;
}

bool Thread::WrapCurrent() {
  return WrapCurrentWithThreadManager(ThreadManager::Instance(), true);
}

void Thread::UnwrapCurrent() {
  // Clears the platform-specific thread-specific storage.
  ThreadManager::Instance()->SetCurrentThread(nullptr);
/* #if defined(WEBRTC_WIN)
  if (thread_ != nullptr) {
    if (!CloseHandle(thread_)) {
      // RTC_LOG_GLE(LS_ERROR) << "When unwrapping thread, failed to close handle.";
    }
    thread_ = nullptr;
    thread_id_ = 0;
  }
#elif defined(WEBRTC_POSIX)
  thread_ = 0;
#endif */
  thread_ref_ = kNullThrd;
}

void Thread::SafeWrapCurrent() {
  WrapCurrentWithThreadManager(ThreadManager::Instance(), false);
}

void Thread::Join() {
  if (!IsRunning())
    return;

  RTC_DCHECK(!IsCurrent());
  if (Current() /* && !Current()->blocking_calls_allowed_ */) {
    /* RTC_LOG(LS_WARNING) << "Waiting for the thread to join, "
                        << "but blocking calls have been disallowed"; */
  }

/* #if defined(WEBRTC_WIN)
  RTC_DCHECK(thread_ != nullptr);
  WaitForSingleObject(thread_, INFINITE);
  CloseHandle(thread_);
  thread_ = nullptr;
  thread_id_ = 0;
#elif defined(WEBRTC_POSIX)
  pthread_join(thread_, nullptr);
  thread_ = 0;
#endif */
  thread_->Stop();
  thread_ref_ = kNullThrd;
}

// static
/* #if defined(WEBRTC_WIN)
DWORD WINAPI Thread::PreRun(LPVOID pv) {
#else
void* Thread::PreRun(void* pv) {
#endif */
void Thread::PreRun(void* pv) {
  Thread* thread = static_cast<Thread*>(pv);
  ThreadManager::Instance()->SetCurrentThread(thread);
  /* ThrdSetName(thread->name_.c_str()); */ // set in platform_thread
  thread->Run();

  ThreadManager::Instance()->SetCurrentThread(nullptr);
/* #if defined(WEBRTC_WIN)
  return 0;
#else
  return nullptr;
#endif */
}  // namespace rtc

void Thread::Run() {
  ProcessMessages(kForever);
}

bool Thread::IsOwned() {
  RTC_DCHECK(IsRunning());
  return owned_;
}

void Thread::Stop() {
  Thread::Quit();
  Join();
}

void Thread::Send(const Location& posted_from,
                  MessageHandler* phandler,
                  uint32_t id,
                  MessageData* pdata) {
  RTC_DCHECK(!IsQuitting());
  if (IsQuitting()) {
    delete pdata; // should we delete here, or let users control itself
    return;
  }

  // Sent messages are sent to the MessageHandler directly, in the context
  // of "thread", like Win32 SendMessage. If in the right context,
  // call the handler directly.
  Message msg;
  msg.posted_from = posted_from;
  msg.phandler = phandler;
  msg.message_id = id;
  msg.pdata = pdata;
  if (IsCurrent()) {
    msg.phandler->OnMessage(&msg);
    return;
  }

  Thread* current_thread = Thread::Current();
#if RTC_DCHECK_IS_ON
  if (current_thread) {
    ThreadManager::Instance()->RegisterSendAndCheckForCycles(current_thread,
                                                             this);
  }
#endif

#if 0 /* defined(USING_SENDLIST) */
  /* AutoThread thread;
  Thread* current_thread = Thread::Current();
  RTC_DCHECK(current_thread != nullptr);  // AutoThread ensures this */

  bool ready = false;
  {
    CritScope cs(&crit_);
    _SendMessage smsg;
    smsg.thread = current_thread;
    smsg.msg = msg;
    smsg.ready = &ready;
    sendlist_.push_back(smsg);
  }

  // Wait for a reply
  WakeUpSocketServer();

  bool waited = false;
  crit_.Enter();
  while (!ready) {
    crit_.Leave();
    // We need to limit "ReceiveSends" to |this| thread to avoid an arbitrary
    // thread invoking calls on the current thread.
    current_thread->ReceiveSendsFromThread(this);
    current_thread->socketserver()->Wait(kForever, false);
    waited = true;
    crit_.Enter();
  }
  crit_.Leave();

  // Our Wait loop above may have consumed some WakeUp events for this
  // MessageQueue, that weren't relevant to this Send.  Losing these WakeUps can
  // cause problems for some SocketServers.
  //
  // Concrete example:
  // Win32SocketServer on thread A calls Send on thread B.  While processing the
  // message, thread B Posts a message to A.  We consume the wakeup for that
  // Post while waiting for the Send to complete, which means that when we exit
  // this loop, we need to issue another WakeUp, or else the Posted message
  // won't be processed in a timely manner.

  if (waited) {
    current_thread->socketserver()->WakeUp();
  }

#else
  // Perhaps down the line we can get rid of this workaround and always require
  // current_thread to be valid when Send() is called.
  std::unique_ptr<rtc::Event> done_event;
  if (!current_thread)
    done_event.reset(new rtc::Event(false, false));

  bool ready = false;
  /* PostTask(webrtc::ToQueuedTask(
      [&msg]() mutable { msg.phandler->OnMessage(&msg); },
      [this, &ready, current_thread, done = done_event.get()] {
        if (current_thread) {
          CritScope cs(&crit_);
          ready = true;
          current_thread->socketserver()->WakeUp();
        } else {
          done->Set();
        }
      })); */
  _SendMessage *smsg = new _SendMessage;
  smsg->thread = current_thread;
  smsg->msg = msg;
  smsg->ready = &ready;
  smsg->done = done_event.get();
  // Though Post takes MessageData by raw pointer (last parameter), it still
  // takes it with ownership.
  Post(RTC_FROM_HERE, &queued_task_handler_, QueuedTaskHandler::kSend, smsg);

  if (current_thread) {
    bool waited = false;
    crit_.Enter();
    while (!ready) {
      crit_.Leave();
      current_thread->socketserver()->Wait(kForever, false);
      waited = true;
      crit_.Enter();
    }
    crit_.Leave();

    // Our Wait loop above may have consumed some WakeUp events for this
    // Thread, that weren't relevant to this Send.  Losing these WakeUps can
    // cause problems for some SocketServers.
    //
    // Concrete example:
    // Win32SocketServer on thread A calls Send on thread B.  While processing
    // the message, thread B Posts a message to A.  We consume the wakeup for
    // that Post while waiting for the Send to complete, which means that when
    // we exit this loop, we need to issue another WakeUp, or else the Posted
    // message won't be processed in a timely manner.

    if (waited) {
      current_thread->socketserver()->WakeUp();
    }
  } else {
    done_event->Wait(rtc::Event::kForever);
  }

#endif
}

/* void Thread::InvokeInternal(const Location& posted_from,
                            rtc::FunctionView<void()> functor) {
  TRACE_EVENT2("webrtc", "Thread::Invoke", "src_file", posted_from.file_name(),
               "src_func", posted_from.function_name());

  class FunctorMessageHandler : public MessageHandler {
   public:
    explicit FunctorMessageHandler(rtc::FunctionView<void()> functor)
        : functor_(functor) {}
    void OnMessage(Message* msg) override { functor_(); }

   private:
    rtc::FunctionView<void()> functor_;
  } handler(functor);

  Send(posted_from, &handler);
} */

#if 0 /* defined(USING_SENDLIST) */
/* void Thread::ReceiveSends() {
  ReceiveSendsFromThread(nullptr);
} */

void Thread::ReceiveSendsFromThread(const Thread* source) {
  // Receive a sent message. Cleanup scenarios:
  // - thread sending exits: We don't allow this, since thread can exit
  //   only via Join, so Send must complete.
  // - thread receiving exits: Wakeup/set ready in Thread::Clear()
  // - object target cleared: Wakeup/set ready in Thread::Clear()
  _SendMessage smsg;

  crit_.Enter();
  while (PopSendMessageFromThread(source, &smsg)) {
    crit_.Leave();

    smsg.msg.phandler->OnMessage(&smsg.msg);

    crit_.Enter();
    *smsg.ready = true;
    smsg.thread->socketserver()->WakeUp();
  }
  crit_.Leave();
}

bool Thread::PopSendMessageFromThread(const Thread* source, _SendMessage* msg) {
  for (std::list<_SendMessage>::iterator it = sendlist_.begin();
       it != sendlist_.end(); ++it) {
    if (it->thread == source || source == nullptr) {
      *msg = *it;
      sendlist_.erase(it);
      return true;
    }
  }
  return false;
}

#else
void Thread::QueuedTaskHandler::OnMessage(Message* msg) {
  RTC_DCHECK(msg);
  /* auto* data = static_cast<ScopedMessageData<webrtc::QueuedTask>*>(msg->pdata);
  std::unique_ptr<webrtc::QueuedTask> task = std::move(data->data()); */
  switch (msg->message_id) {
    case kSend: { // without this brace, error: conflicting declaration ... will happen
      _SendMessage *smsg = (_SendMessage *)msg->pdata;
      smsg->msg.phandler->OnMessage(&smsg->msg);
      if (smsg->thread) {
        CritScope cs(&smsg->thread->crit_);
        *smsg->ready = true;
        smsg->thread->socketserver()->WakeUp();
      } else {
        smsg->done->Set();
      }
      break;
    }
    /* case kSetDispatchWarningMs: {
      _SetDispatchWarningMsMessage *smsg = (_SetDispatchWarningMsMessage *)msg->pdata;
      if (smsg->thread) {
        smsg->thread->SetDispatchWarningMs(smsg->deadline);
      } else {
        // what to do here?
      }
      break;
    } */
    case kSleepMs: {
      _SleepMsMessage *smsg = (_SleepMsMessage *)msg->pdata;
      if (smsg->thread) {
        smsg->thread->SleepMs(smsg->milliseconds);
      } else {
        // what to do here?
      }
      break;
    }
  }
  // Thread expects handler to own Message::pdata when OnMessage is called
  // Since MessageData is no longer needed, delete it.
  delete msg->pdata;

  /* // QueuedTask interface uses Run return value to communicate who owns the
  // task. false means QueuedTask took the ownership.
  if (!task->Run())
    task.release(); */
}

#endif

/* void Thread::Delete() {
  Stop();
  delete this;
} */

bool Thread::IsProcessingMessages() {
  return (owned_ || IsCurrent()) && !IsQuitting();
}

void Thread::Clear(MessageHandler* phandler,
                   uint32_t id,
                   MessageList* removed) {
  CritScope cs(&crit_);
  ClearInternal(phandler, id, removed);
}

bool Thread::ProcessMessages(int cmsLoop) {
  // Using ProcessMessages with a custom clock for testing and a time greater
  // than 0 doesn't work, since it's not guaranteed to advance the custom
  // clock's time, and may get stuck in an infinite loop.
  RTC_DCHECK(cmsLoop == 0 || cmsLoop == kForever);
  int64_t msEnd = (kForever == cmsLoop) ? 0 : TimeAfter(cmsLoop);
  int cmsNext = cmsLoop;

  while (true) {
    Message msg;
    if (!Get(&msg, cmsNext))
      return !IsQuitting();
    Dispatch(&msg);

    if (cmsLoop != kForever) {
      cmsNext = static_cast<int>(TimeUntil(msEnd));
      if (cmsNext < 0)
        return true;
    }
  }
}

bool Thread::WrapCurrentWithThreadManager(ThreadManager* thread_manager,
                                          bool need_synchronize_access) {
  RTC_DCHECK(!IsRunning());

/* #if defined(WEBRTC_WIN)
  if (need_synchronize_access) {
    // We explicitly ask for no rights other than synchronization.
    // This gives us the best chance of succeeding.
    thread_ = OpenThread(SYNCHRONIZE, FALSE, GetCurrentThreadId());
    if (!thread_) {
      // RTC_LOG_GLE(LS_ERROR) << "Unable to get handle to thread.";
      return false;
    }
    thread_id_ = GetCurrentThreadId();
  }
#elif defined(WEBRTC_POSIX)
  RTC_UNUSED(need_synchronize_access);
  thread_ = pthread_self();
#endif */
  RTC_UNUSED(need_synchronize_access); // TODO imply synchronization in PlatformThread
  thread_ref_ = thread_->GetThreadRef();
  owned_ = false;
  thread_manager->SetCurrentThread(this);
  return true;
}

bool Thread::IsRunning() {
/* #if defined(WEBRTC_WIN)
  return thread_ != nullptr;
#elif defined(WEBRTC_POSIX)
  return thread_ != 0;
#endif */
  /* if (owned_) {
    return ((thread_ref_ != kNullThrd) && (thread_ != nullptr)
      && thread_->IsRunning());
  }
  else */ {
    // |thread_| will not running if not call |Thread::Start()|
    // only call |ThreadManager::WrapCurrentThread()| also works
    return (thread_ref_ != kNullThrd);
  }
}

#if 0 /* defined(USING_SENDLIST) */
AutoThread::AutoThread()
    : Thread(SocketServer::CreateDefault(), /*do_init=*/false) {
  if (!ThreadManager::Instance()->CurrentThread()) {
    // DoInit registers with ThreadManager. Do that only if we intend to
    // be rtc::Thread::Current(), otherwise ProcessAllMessageQueuesInternal will
    // post a message to a queue that no running thread is serving.
    DoInit();
    ThreadManager::Instance()->SetCurrentThread(this);
  }
}

AutoThread::~AutoThread() {
  Stop();
  DoDestroy();
  if (ThreadManager::Instance()->CurrentThread() == this) {
    ThreadManager::Instance()->SetCurrentThread(nullptr);
  }
}

AutoSocketServerThread::AutoSocketServerThread(SocketServer* ss)
    : Thread(ss, /*do_init=*/false) {
  DoInit();
  old_thread_ = ThreadManager::Instance()->CurrentThread();
  // Temporarily set the current thread to nullptr so that we can keep checks
  // around that catch unintentional pointer overwrites.
  rtc::ThreadManager::Instance()->SetCurrentThread(nullptr);
  rtc::ThreadManager::Instance()->SetCurrentThread(this);
  if (old_thread_) {
    ThreadManager::Remove(old_thread_);
  }
}

AutoSocketServerThread::~AutoSocketServerThread() {
  RTC_DCHECK(ThreadManager::Instance()->CurrentThread() == this);
  // Some tests post destroy messages to this thread. To avoid memory
  // leaks, we have to process those messages. In particular
  // P2PTransportChannelPingTest, relying on the message posted in
  // cricket::Connection::Destroy.
  ProcessMessages(0);
  // Stop and destroy the thread before clearing it as the current thread.
  // Sometimes there are messages left in the Thread that will be
  // destroyed by DoDestroy, and sometimes the destructors of the message and/or
  // its contents rely on this thread still being set as the current thread.
  Stop();
  DoDestroy();
  rtc::ThreadManager::Instance()->SetCurrentThread(nullptr);
  rtc::ThreadManager::Instance()->SetCurrentThread(old_thread_);
  if (old_thread_) {
    ThreadManager::Add(old_thread_);
  }
}

#endif

}  // namespace rtc
