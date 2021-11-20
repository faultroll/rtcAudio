
#pragma once

#if 0 /* detect which to use */
    // c11: stdalign aligned_alloc
    #undef _ALIGN_C_USE_STD
    #undef _ALIGN_C_USE_POSIX
    #undef _ALIGN_C_USE_WIN
    #undef _ALIGN_C_USE_NONE
    // c11: stdatomic
    #undef _ATOMIC_C_USE_STD
    #undef _ATOMIC_C_USE_POSIX
    #undef _ATOMIC_C_USE_WIN
    #undef _ATOMIC_C_USE_NONE
    // c11: threads stdnoreturn
    #undef _THREAD_C_USE_STD
    #undef _THREAD_C_USE_POSIX
    #undef _THREAD_C_USE_WIN
    #undef _THREAD_C_USE_NONE
    // c11: time
    #undef _TIME_C_USE_STD
    #undef _TIME_C_USE_POSIX
    #undef _TIME_C_USE_WIN
    #undef _TIME_C_USE_NONE
#else /* edit yourself */
    #if defined(WEBRTC_WIN)
        #define _ALIGN_C_USE_WIN
    #elif defined(WEBRTC_POSIX)
        #define _ALIGN_C_USE_POSIX
    #else
        #define _ALIGN_C_USE_NONE
    #endif // defined(WEBRTC_WIN)
    /**
    * std::unique_ptr is in <memory> which includes <shared_ptr_atomic.h>
    * however, it only implements the atomic_flag and cannot use <stdatomics.h>
    *
    */
    #if defined(WEBRTC_WIN)
        #define _ATOMIC_C_USE_WIN
    #elif defined(WEBRTC_POSIX)
        #define _ATOMIC_C_USE_POSIX
    #else
        #define _ATOMIC_C_USE_NONE
    #endif // defined(WEBRTC_WIN)
    #if defined(WEBRTC_WIN)
        #define _THREAD_C_USE_WIN
    #elif defined(WEBRTC_POSIX)
        #define _THREAD_C_USE_POSIX
    #else
        #define _THREAD_C_USE_NONE
    #endif // defined(WEBRTC_WIN)
    /**
    *  |timespec_get| re-declare, so we use <time.h> here
    *
    **/
    #if defined(WEBRTC_WIN)
        #define _TIME_C_USE_WIN
    #elif defined(WEBRTC_POSIX)
        #define _TIME_C_USE_STD
    #else
        #define _TIME_C_USE_NONE
    #endif // defined(WEBRTC_WIN)
#endif
