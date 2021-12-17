
#include "thread_c.h"

#if defined(_THREAD_C_USE_STD)
    // #include <threads.h>
#else
    #include <stdlib.h>
    /**
     * thrd
     *
     **/
    static int thrd_start_wrapper_func_helper(void *wrapper);
    #if defined(_THREAD_C_USE_POSIX)
        #include <sched.h> // sched_yield()
        #if defined(__linux__) || defined(__LINUX__) || defined(linux) || defined(LINUX)
            #define _THREAD_C_USE_LINUX
        #endif /* defined(__linux__) */
        #if defined(_THREAD_C_USE_LINUX)
            #include <sys/prctl.h> // thrd_set_name
            #include <sys/syscall.h>
        #endif
        #include <math.h> // fmax
        static void *thrd_start_wrapper_func(void *wrapper)
        {
            return (void *)(intptr_t)thrd_start_wrapper_func_helper(wrapper);
        }
        // Cannot use |static thrd_t thrd_create_helper(void *ti)|
        // race will occur if use |thr| in |thrd_start_wrapper_func|
        // for |thr| will be |thrd_null| if |thrd_create_helper| is not return
        // but |thrd_start_wrapper_func| thread uses |thr|
        static void thrd_create_helper(thrd_t *thr, void *ti)
        {
            pthread_attr_t attr;
            pthread_attr_init(&attr);
            // Set the stack stack size to 1M.
            pthread_attr_setstacksize(&attr, 1024 * 1024);
            if (pthread_create(thr, &attr, &thrd_start_wrapper_func, (void *)ti) != 0) {
                *thr = thrd_null;
            }
            pthread_attr_destroy(&attr);
        }
        thrd_t thrd_current(void)
        {
            return pthread_self();
        }
        int thrd_detach(thrd_t thr)
        {
            return pthread_detach(thr);
        }
        // Non-zero value if thr0 and thr1 refer to the same value, ​0​ otherwise.
        int thrd_equal(thrd_t thr0, thrd_t thr1)
        {
            return (0 == pthread_equal(thr0, thr1)) ? 1 : 0;
        }
        noreturn void thrd_exit(int res)
        {
            pthread_exit((void *)(intptr_t)res);
        }
        int thrd_join(thrd_t thr, int *res)
        {
            void *pres;
            if (pthread_join(thr, &pres) != 0) {
                return thrd_error;
            }
            if (res != NULL) {
                *res = (int)(intptr_t)pres;
            }
            return thrd_success;
        }
        // ​0​ on successful sleep, -1 if a signal occurred, other negative value if an error occurred.
        int thrd_sleep(const struct timespec *duration, struct timespec *remaining)
        {
            return nanosleep(duration, remaining);
        }
        void thrd_yield(void)
        {
            sched_yield(); // nanosleep(&ts_null, NULL);
        }
        int thrd_set_priority(thrd_t thr, int prio)
        {
            const int policy = SCHED_RR;
            const int min_prio = sched_get_priority_min(policy);
            const int max_prio = sched_get_priority_max(policy);
            if (min_prio == -1 || max_prio == -1)
                return thrd_error;

            if (max_prio - min_prio <= 2)
                return thrd_error;

            // Convert webrtc priority to system priorities:
            struct sched_param param;
            const int top_prio = max_prio - 1;
            const int low_prio = min_prio + 1;
            switch (prio) {
                case thrd_priority_low:
                    param.sched_priority = low_prio;
                    break;
                case thrd_priority_normal:
                    // The -1 ensures that the |thrd_priority_high| is always
                    // greater or equal to |thrd_priority_normal|.
                    param.sched_priority = (low_prio + top_prio - 1) / 2;
                    break;
                case thrd_priority_high:
                    param.sched_priority = /* std::max */fmax(top_prio - 2, low_prio);
                    break;
                case thrd_priority_highest:
                    param.sched_priority = /* std::max */fmax(top_prio - 1, low_prio);
                    break;
                case thrd_priority_realtime:
                    param.sched_priority = top_prio;
                    break;
            }
            return pthread_setschedparam(thr, policy, &param);
        }
        void thrd_set_name(const char *name)
        {
        #if defined(_THREAD_C_USE_LINUX)
            prctl(PR_SET_NAME, (unsigned long)(uintptr_t)(name));  // NOLINT
        #endif
        }
    #elif defined(_THREAD_C_USE_WIN)
        static DWORD WINAPI thrd_start_wrapper_func(LPVOID wrapper)
        {
            // The GetLastError() function only returns valid results when it is called
            // after a Win32 API function that returns a "failed" result. A crash dump
            // contains the result from GetLastError() and to make sure it does not
            // falsely report a Windows error we call SetLastError here.
            SetLastError(ERROR_SUCCESS);

            return (DWORD)thrd_start_wrapper_func_helper(wrapper);
        }
        static void thrd_create_helper(thrd_t *thr, void *ti)
        {
            // Create the thread
            // See bug 2902 for background on STACK_SIZE_PARAM_IS_A_RESERVATION.
            // Set the reserved stack stack size to 1M, which is the default on Windows
            // and Linux.
            *thr = CreateThread(NULL, 1024 * 1024, &thrd_start_wrapper_func, (LPVOID) ti,
                                STACK_SIZE_PARAM_IS_A_RESERVATION, NULL);
        }
        thrd_t thrd_current(void)
        {
            return GetCurrentThread();
        }
        int thrd_detach(thrd_t thr)
        {
            // https://stackoverflow.com/questions/12744324/how-to-detach-a-thread-on-windows-c#answer-12746081
            return CloseHandle(thr);
        }
        int thrd_equal(thrd_t thr0, thrd_t thr1)
        {
            /* // OwningThread has type HANDLE but actually contains the Thread ID:
            // http://stackoverflow.com/questions/12675301/why-is-the-owningthread-member-of-critical-section-of-type-handle-when-it-is-de
            // Converting through size_t avoids the VS 2015 warning C4312: conversion from
            // 'type1' to 'type2' of greater size
            return crit_.OwningThread ==
                reinterpret_cast<HANDLE>(static_cast<size_t>(GetCurrentThreadId())); */
            return (GetThreadId(thr0) == GetThreadId(thr1)) ? 1 : 0;
        }
        noreturn void thrd_exit(int res)
        {
            ExitThread((DWORD)res);
        }
        int thrd_join(thrd_t thr, int *res)
        {
            DWORD dwRes;
            if (WaitForSingleObject(thr, INFINITE) == WAIT_FAILED) {
                return thrd_error;
            }
            if (res != NULL) {
                if (GetExitCodeThread(thr, &dwRes) == 0) {
                    return thrd_error;
                }
                *res = (int)dwRes;
            }
            CloseHandle(thr);
            return thrd_success;
        }
        int thrd_sleep(const struct timespec *duration, struct timespec *remaining)
        {
            SleepEx((DWORD)(duration->tv_sec * 1000 +
                    duration->tv_nsec / 1000000 +
                    (((duration->tv_nsec % 1000000) == 0) ? 0 : 1)),
                    TRUE);
            return 0;
        }
        void thrd_yield(void)
        {
            // Alertable sleep to permit RaiseFlag to run and update |stop_|.
            SleepEx(0, true); // Sleep(0);
        }
        int thrd_set_priority(thrd_t thr, int prio)
        {
            return SetThreadPrio(thr, prio);
        }
        void thrd_set_name(const char *name)
        {
            struct {
                DWORD dwType;
                LPCSTR szName;
                DWORD dwThreadID;
                DWORD dwFlags;
            } threadname_info = {0x1000, name, static_cast<DWORD>(-1), 0};

            __try {
                RaiseException(0x406D1388, 0, sizeof(threadname_info) / sizeof(DWORD),
                            reinterpret_cast<ULONG_PTR *>(&threadname_info));
            } __except (EXCEPTION_EXECUTE_HANDLER) {  // NOLINT
            }
        }
    #elif defined(_THREAD_C_USE_NONE)
        // use coroutine
        // rtos |taskDelay| ...
        // #include "pt.h"
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |thrd|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // Information to pass to the new thread (what to run).
    typedef struct {
        thrd_start_t func_; // Pointer to the function to be executed.
        void *arg_;         // Function argument for the thread function.
    } thrd_start_wrapper_args_t;
    static int thrd_start_wrapper_func_helper(void *wrapper)
    {
        thrd_start_t fun;
        void *arg;
        int  result = thrd_error;

        // Get thread startup information
        thrd_start_wrapper_args_t *ti = (thrd_start_wrapper_args_t *) wrapper;
        fun = ti->func_;
        arg = ti->arg_;

        // The thread is responsible for freeing the startup information
        free((void *)ti);

        // Call the actual client thread function
        if (fun)
            result = fun(arg);

        return result;
    }
    int thrd_create(thrd_t *thr, thrd_start_t func, void *arg)
    {
        // Fill out the thread startup information
        // (passed to the thread wrapper, which will eventually free it)
        thrd_start_wrapper_args_t *ti = (thrd_start_wrapper_args_t *)malloc(sizeof(thrd_start_wrapper_args_t));
        if (ti == NULL) {
            return thrd_nomem;
        }
        ti->func_ = func;
        ti->arg_ = arg;

        thrd_create_helper(thr, (void *)ti);

        // Did we fail to create the thread?
        if (thrd_null == *thr) {
            free(ti);
            return thrd_error;
        }

        return thrd_success;
    }
    /**
     * tss
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        int tss_create(tss_t *key, tss_dtor_t dtor)
        {
            return pthread_key_create(key, dtor);
        }
        void tss_delete(tss_t key)
        {
            pthread_key_delete(key);
        }
        void *tss_get(tss_t key)
        {
            return pthread_getspecific(key);
        }
        int tss_set(tss_t key, void *val)
        {
            return pthread_setspecific(key, val);
        }
    #elif defined(_THREAD_C_USE_WIN)
        int tss_create(tss_t *key, tss_dtor_t dtor)
        {
            if ((*key = TlsAlloc()) == tss_null) {
                return thrd_error
            } else {
                return thrd_success;
            }
        }
        void tss_delete(tss_t key)
        {
            TlsFree(key);
        }
        void *tss_get(tss_t key)
        {
            return TlsGetValue(key);
        }
        int tss_set(tss_t key, void *val)
        {
            return TlsSetValue(key, val);
        }
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |tss|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    /**
     * mtx
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        int mtx_init(mtx_t *mtx, int type)
        {
            int result;
            pthread_mutexattr_t mutex_attribute;
            pthread_mutexattr_init(&mutex_attribute);
            if (type & mtx_recursive) {
                pthread_mutexattr_settype(&mutex_attribute, PTHREAD_MUTEX_RECURSIVE);
            }
            result = pthread_mutex_init(mtx, &mutex_attribute);
            pthread_mutexattr_destroy(&mutex_attribute);
            return result;
        }
        void mtx_destroy(mtx_t *mtx)
        {
            pthread_mutex_destroy(mtx);
        }
        int mtx_lock(mtx_t *mtx)
        {
            return pthread_mutex_lock(mtx);
        }
        int mtx_timedlock(mtx_t *mtx, const struct timespec *ts)
        {
            return pthread_mutex_timedlock(mtx, ts);
        }
        int mtx_trylock(mtx_t *mtx)
        {
            return pthread_mutex_trylock(mtx);
        }
        int mtx_unlock(mtx_t *mtx)
        {
            return pthread_mutex_unlock(mtx);
        }
    #elif defined(_THREAD_C_USE_WIN)
        int mtx_init(mtx_t *mtx, int type)
        {
            return InitializeCriticalSection(mtx);
        }
        void mtx_destroy(mtx_t *mtx)
        {
            DeleteCriticalSection(mtx);
        }
        int mtx_lock(mtx_t *mtx)
        {
            return EnterCriticalSection(mtx);
        }
        int mtx_timedlock(mtx_t *mtx, const struct timespec *ts)
        {
            return thrd_error;
        }
        int mtx_trylock(mtx_t *mtx)
        {
            return TryEnterCriticalSection(mtx);
        }
        int mtx_unlock(mtx_t *mtx)
        {
            return LeaveCriticalSection(mtx);
        }
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |mtx|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    /**
     * cnd
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        int cnd_init(cnd_t *cond)
        {
            return pthread_cond_init(cond, NULL);
        }
        void cnd_destroy(cnd_t *cond)
        {
            pthread_cond_destroy(cond);
        }
        int cnd_signal(cnd_t *cond)
        {
            return pthread_cond_signal(cond);
        }
        int cnd_broadcast(cnd_t *cond)
        {
            return pthread_cond_broadcast(cond);
        }
        int cnd_wait(cnd_t *cond, mtx_t *mtx)
        {
            return pthread_cond_wait(cond, mtx);
        }
        int cnd_timedwait(cnd_t *cond, mtx_t *mtx, const struct timespec *ts)
        {
            return pthread_cond_timedwait(cond, mtx, ts);
        }
    #elif defined(_THREAD_C_USE_WIN)
        int cnd_init(cnd_t *cond)
        {
            cond->waiter_count_ = 0;
            // Init critical section
            InitializeCriticalSection(&cond->waiter_mutex_);
            // Init events
            cond->events_[cnd_event_signal] = CreateEvent(NULL, false, false, NULL);
            if (cond->events_[cnd_event_signal] == NULL) {
                cond->events_[cnd_event_broadcast] = NULL;
                return thrd_error;
            }
            cond->events_[cnd_event_broadcast] = CreateEvent(NULL, true, false, NULL);
            if (cond->events_[cnd_event_broadcast] == NULL) {
                CloseHandle(cond->events_[cnd_event_signal]);
                cond->events_[cnd_event_signal] = NULL;
                return thrd_error;
            }
            return thrd_success;
        }
        void cnd_destroy(cnd_t *cond)
        {
            if (cond->events_[cnd_event_signal] != NULL) {
                CloseHandle(cond->events_[cnd_event_signal]);
            }
            if (cond->events_[cnd_event_broadcast] != NULL) {
                CloseHandle(cond->events_[cnd_event_broadcast]);
            }
            DeleteCriticalSection(&cond->waiter_mutex_);
        }
        int cnd_signal(cnd_t *cond)
        {
            return thrd_error;
        }
        int cnd_broadcast(cnd_t *cond)
        {
            return thrd_error;
        }
        int cnd_wait(cnd_t *cond, mtx_t *mtx)
        {
            return thrd_error;
        }
        int cnd_timedwait(cnd_t *cond, mtx_t *mtx, const struct timespec *ts)
        {
            return thrd_error;
        }
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |cnd|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    /**
     * once
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        // #define call_once(flag,func) pthread_once(flag,func)
    #elif defined(_THREAD_C_USE_WIN)
        void call_once(once_flag *flag, void (*func)(void))
        {
            /* The idea here is that we use a spin lock (via the
            InterlockedCompareExchange function) to restrict access to the
            critical section until we have initialized it, then we use the
            critical section to block until the callback has completed
            execution. */
            while (flag->status < 3) {
                switch (flag->status) {
                    case 0:
                        if (InterlockedCompareExchange(&(flag->status), 1, 0) == 0) {
                            InitializeCriticalSection(&(flag->lock));
                            EnterCriticalSection(&(flag->lock));
                            flag->status = 2;
                            func();
                            flag->status = 3;
                            LeaveCriticalSection(&(flag->lock));
                            return;
                        }
                        break;
                    case 1:
                        break;
                    case 2:
                        EnterCriticalSection(&(flag->lock));
                        LeaveCriticalSection(&(flag->lock));
                        break;
                }
            }
        }
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |once|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
#endif /* defined(_THREAD_C_USE_STD) */
