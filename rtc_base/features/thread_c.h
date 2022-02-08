
// c11: threads stdnoreturn
// https://en.cppreference.com/w/c/thread
// https://github.com/tinycthread/tinycthread
// https://github.com/cdschreiber/c11
// https://github.com/zserge/pt

#ifndef _THREAD_C_H
#define _THREAD_C_H

#include "features_config.h"

#if !defined(_THREAD_C_USE_STD) \
    && !defined(_THREAD_C_USE_POSIX) \
    && !defined(_THREAD_C_USE_WIN) \
    && !defined(_THREAD_C_USE_NONE)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L) \
        && !defined(__STDC_NO_THREADS__)
        #define _THREAD_C_USE_STD
    #elif defined(__GNUC__) || defined(__clang__) // defined(__linux__) || defined(__APPLE__)
        #define _THREAD_C_USE_POSIX
    #elif defined(_MSC_VER) // defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
        #define _THREAD_C_USE_WIN
    #else
        #define _THREAD_C_USE_NONE
    #endif /* !defined(__STDC_NO_THREADS__) */
#endif /* !defined(_THREAD_C_USE_STD) && ... */

// before include header files
#if defined(_THREAD_C_USE_POSIX)
    #undef _POSIX_C_SOURCE
    #define _POSIX_C_SOURCE 200809L
#endif /* defined(_THREAD_C_USE_POSIX) */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_THREAD_C_USE_STD)
    // #include <time.h>
    // #include <stdnoreturn.h>
    #include <threads.h>
#else
    // #include <stdlib.h>
    /**
     * timespec
     *
     **/
    #if defined(_THREAD_C_USE_POSIX)
        #include <time.h>
        // #include <sys/time.h>
    #elif defined(_THREAD_C_USE_WIN)
        #ifndef WIN32_LEAN_AND_MEAN
            #define WIN32_LEAN_AND_MEAN
        #endif /* WIN32_LEAN_AND_MEAN */
        #include <windows.h>
        #include <mmsystem.h>
        #include <sys/timeb.h>
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
    #elif defined(_THREAD_C_USE_NONE)
    #else
        #error "thread_c: unknown branch in |timespec|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // helper
    static const struct timespec ts_null = {0, 0};
    /**
     * stdnoreturn
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        #ifndef noreturn
        // something is wrong with |__cplusplus|
        // error: '__noreturn__' was not declared in this scope
        #if defined(__cplusplus)
            #define noreturn 
        #else
            #define noreturn __attribute__ ((__noreturn__))
        #endif /* defined(__cplusplus) */
        #endif /* noreturn */
    #elif defined(_THREAD_C_USE_WIN)
        #ifndef noreturn
            #define noreturn __declspec(noreturn)
        #endif /* noreturn */
    #elif defined(_THREAD_C_USE_NONE)
        #ifndef noreturn
            #define noreturn 
        #endif /* noreturn */
    #else
        #error "thread_c: unknown branch in |noreturn|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    /**
     * thrd
     *
     **/
    // Any thread that is started with the |thrd_create| function must be
    // started through a function of this type.
    typedef int (*thrd_start_t)(void *arg);
    #if defined(_THREAD_C_USE_POSIX)
        #include <pthread.h>
        #include <unistd.h>
        // Function return values
        enum {
            thrd_success    = 0,  // The requested operation succeeded
            thrd_nomem      = -4, // The requested operation failed because it was unable to allocate memory
            thrd_timedout   = -3, // The time specified in the call was reached without acquiring the requested resource
            thrd_busy       = -2, // The requested operation failed because a tesource requested by a test and return function is already in use
            thrd_error      = -1, // The requested operation failed
        };
        // just set |thrd_success| to different in different platform
        // #define thrd_check_return(result) do { return ((0 == (result)) ? thrd_success : thrd_error); } while(0)
        typedef pthread_t thrd_t;
        static const thrd_t thrd_null = (thrd_t)(0);
        enum {
            thrd_priority_low = 1,
            thrd_priority_normal = 2,
            thrd_priority_high = 3,
            thrd_priority_highest = 4,
            thrd_priority_realtime = 5,
        };
    #elif defined(_THREAD_C_USE_WIN)
        // Include winsock2.h before including <windows.h> to maintain consistency with
        // win32.h.  We can't include win32.h directly here since it pulls in
        // headers such as basictypes.h which causes problems in Chromium where webrtc
        // exists as two separate projects, webrtc and libjingle.
        #include <winsock2.h>
        #include <windows.h>
        #include <sal.h>  // must come after windows headers.
        // Function return values
        enum {
            thrd_success    = ERROR_SUCCESS /* 1 */,  // The requested operation succeeded
            thrd_nomem      = -4, // The requested operation failed because it was unable to allocate memory
            thrd_timedout   = -3, // The time specified in the call was reached without acquiring the requested resource
            thrd_busy       = -2, // The requested operation failed because a tesource requested by a test and return function is already in use
            thrd_error      = -1, // The requested operation failed
        };
        // #define thrd_check_return(result) do { return (((result) != 0) ? thrd_success : thrd_error); } while(0)
        typedef HANDLE thrd_t;
        static const thrd_t thrd_null = (thrd_t)(0);
        enum {
            thrd_priority_low = THREAD_PRIORITY_BELOW_NORMAL,
            thrd_priority_normal = THREAD_PRIORITY_NORMAL,
            thrd_priority_high = THREAD_PRIORITY_ABOVE_NORMAL,
            thrd_priority_highest = THREAD_PRIORITY_HIGHEST,
            thrd_priority_realtime = THREAD_PRIORITY_TIME_CRITICAL,
        };
    #elif defined(_THREAD_C_USE_NONE)
        // use coroutine
        // rtos |taskDelay| ...
        // #include "pt.h"
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |thrd|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // Create a new thread
    // extern int thrd_create(thrd_t *thr, thrd_start_t func, void *arg);
    // Retrieves a reference to the current thread. On Windows, this is the same
    // as CurrentThreadId. On other platforms it's the pthread_t returned by
    // pthread_self().
    // extern thrd_t thrd_current(void);
    // Dispose of any resources allocated to the thread when that thread exits
    // extern int thrd_detach(thrd_t thr);
    // Compares two thread identifiers for equality.
    // Non-zero value if thr0 and thr1 refer to the same value, ​0​ otherwise.
    // extern int thrd_equal(thrd_t thr0, thrd_t thr1);
    // Terminate execution of the calling thread
    // extern noreturn void thrd_exit(int res);
    // Wait for a thread to terminate
    // extern int thrd_join(thrd_t thr, int *res);
    // ​0​ on successful sleep, -1 if a signal occurred, other negative value if an error occurred.
    // extern int thrd_sleep(const struct timespec *duration, struct timespec *remaining);
    // Yield execution to another thread. Permit other threads to run,
    // even if current thread would ordinarily continue to run.
    // extern void thrd_yield(void);
    // Set the priority of the thread. Must be called when thread is running.
    // extern int thrd_set_priority(thrd_t thr, int prio);
    // Sets the current thread name.
    // extern void thrd_set_name(const char *name);
    /**
     * tss
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        #ifndef thread_local
            #define thread_local __thread
        #endif /* thread_local */
        // Destructor function for a TsS
        typedef void (*tss_dtor_t)(void *val);
        // TLs(Thread Local-storage) or TsS(Thread-specific Storage)
        typedef pthread_key_t tss_t;
        static const tss_t tss_null = (tss_t)(0);
    #elif defined(_THREAD_C_USE_WIN)
        #ifndef thread_local
            #define thread_local __declspec(thread)
        #endif /* thread_local */
        typedef DWORD tss_t;
        static const tss_t tss_null = (tss_t)(TLS_OUT_OF_INDEXES); 
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |tss|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // Create a TsS
    // extern int tss_create(tss_t *key, tss_dtor_t dtor);
    // Delete a TsS
    // extern void tss_delete(tss_t key);
    // Get the value for a TsS
    // extern void *tss_get(tss_t key);
    // Set the value for a TsS
    // extern int tss_set(tss_t key, void *val);
    /**
     * mtx
     * 
     **/
    // Bit-mask
    enum {
        mtx_plain = 0/* unspecified */, // default
        mtx_recursive = 1 << 0/* unspecified */, // recursive
        mtx_timed = 1 << 1/* unspecified */, // supports timeout
    };
    #if defined(_THREAD_C_USE_POSIX)
        // Mutex
        typedef pthread_mutex_t mtx_t;
        // static const mtx_t mtx_null = (mtx_t)(0);
    #elif defined(_THREAD_C_USE_WIN)
        typedef CRITICAL_SECTION mtx_t;
        // static const mtx_t mtx_null = (mtx_t)(0);
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |mtx|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // Create a mutex object
    // extern int mtx_init(mtx_t *mtx, int type);
    // Release any resources used by the given mutex
    // extern void mtx_destroy(mtx_t *mtx);
    // Lock the given mutex
    // Blocks until the given mutex can be locked. If the mutex is non-recursive, and
    // the calling thread already has a lock on the mutex, this call will block forever.
    // extern int mtx_lock(mtx_t *mtx);
    // Lock the given mutex, or block until a specific point in time.
    // Blocks until either the given mutex can be locked, or the specified TIME_UTC
    // based time.
    // extern int mtx_timedlock(mtx_t *mtx, const struct timespec *ts);
    // Try to lock the given mutex
    // The specified mutex shall support either test and return or timeout. If the
    // mutex is already locked, the function returns without blocking.
    // extern int mtx_trylock(mtx_t *mtx);
    // Unlock the given mutex
    // extern int mtx_unlock(mtx_t *mtx);
    /**
     * cnd
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        // Condition variable
        typedef pthread_cond_t cnd_t;
    #elif defined(_THREAD_C_USE_WIN)
        enum {
            cnd_event_signal = 0, // signal
            cnd_event_broadcast, // broadcast
            cnd_event_num,
        };
        typedef struct {
            HANDLE events_[cnd_event_num];  // Signal and broadcast event HANDLEs.
            size_t waiter_count_;           // Count of the number of waiters.
            CRITICAL_SECTION waiter_mutex_; // Serialize access to |waiter_count_|
        } cnd_t;
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |cnd|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
    // Create a condition variable object
    // extern int cnd_init(cnd_t *cond);
    // Release any resources used by the given condition variable
    // extern void cnd_destroy(cnd_t *cond);
    // Signal a condition variable
    // Unblocks one of the threads that are blocked on the given condition variable
    // at the time of the call. If no threads are blocked on the condition variable
    // at the time of the call, the function does nothing and return success.
    // extern int cnd_signal(cnd_t *cond);
    // Broadcast a condition variable.
    // Unblocks all of the threads that are blocked on the given condition variable
    // at the time of the call. If no threads are blocked on the condition variable
    // at the time of the call, the function does nothing and return success.
    // extern int cnd_broadcast(cnd_t *cond);
    // Wait for a condition variable to become signaled.
    // The function atomically unlocks the given mutex and endeavors to block until
    // the given condition variable is signaled by a call to cnd_signal or to cnd_broadcast.
    // When the calling thread becomes unblocked it locks the mutex before it returns.
    // extern int cnd_wait(cnd_t *cond, mtx_t *mtx);
    // Wait for a condition variable to become signaled.
    // The function atomically unlocks the given mutex and endeavors to block until
    // the given condition variable is signaled by a call to cnd_signal or to
    // cnd_broadcast, or until after the specified time. When the calling thread
    // becomes unblocked it locks the mutex before it returns.
    // extern int cnd_timedwait(cnd_t *cond, mtx_t *mtx, const struct timespec *ts);
    /**
     * once
     * 
     **/
    #if defined(_THREAD_C_USE_POSIX)
        #define once_flag pthread_once_t
        #define ONCE_FLAG_INIT PTHREAD_ONCE_INIT
        #define call_once(flag,func) pthread_once(flag,func)
    #elif defined(_THREAD_C_USE_WIN)
        typedef struct {
            volatile LONG status;
            CRITICAL_SECTION lock;
        } once_flag;
        #define ONCE_FLAG_INIT {0,}
        // extern void call_once(once_flag *flag, void (*func)(void));
    #elif defined(_THREAD_C_USE_NONE)
        #error "thread_c: current not support |_THREAD_C_USE_NONE|"
    #else
        #error "thread_c: unknown branch in |once|"
    #endif /* defined(_THREAD_C_USE_POSIX) */
#endif /* defined(_THREAD_C_USE_STD) */

#if defined(__cplusplus)
}
#endif

#include "thread_c_inl.h"

#endif /* _THREAD_C_H */
