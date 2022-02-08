
// c11: time
// https://en.cppreference.com/w/c/chrono
// https://github.com/open-webrtc-toolkit/owt-deps-webrtc
// https://github.com/cdschreiber/c11

#ifndef _TIME_C_H
#define _TIME_C_H

#include "features_config.h"

#if !defined(_TIME_C_USE_STD) \
    && !defined(_TIME_C_USE_POSIX) \
    && !defined(_TIME_C_USE_WIN) \
    && !defined(_TIME_C_USE_NONE)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
        #define _TIME_C_USE_STD
    #elif defined(__GNUC__) || defined(__clang__) // defined(__linux__) || defined(__APPLE__)
        #define _TIME_C_USE_POSIX
    #elif defined(_MSC_VER) // defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
        #define _TIME_C_USE_WIN
    #else
        #define _TIME_C_USE_NONE
    #endif /* __STDC_VERSION__ >= 201112L */
#endif /* !defined(_TIME_C_USE_STD) && ... */

// before include header files
#if defined(_TIME_C_USE_POSIX)
    #undef _POSIX_C_SOURCE
    #define _POSIX_C_SOURCE 200809L
#endif /* defined(_TIME_C_USE_POSIX) */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_TIME_C_USE_STD)
    #include <time.h>
#else
    #if defined(_TIME_C_USE_POSIX)
        #include <time.h>
        // #include <sys/time.h>
    #elif defined(_TIME_C_USE_WIN)
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
    #elif defined(_TIME_C_USE_NONE)
    #else
        #error "time_c: unknown branch in |timespec|"
    #endif /* defined(_TIME_C_USE_POSIX) */
#endif /* defined(_TIME_C_USE_STD) */

// helper
#define MS_PER_SEC  INT64_C(1000)
#define US_PER_SEC  INT64_C(1000000)
#define NS_PER_SEC  INT64_C(1000000000)
#define US_PER_MS   INT64_C(1000)       // (US_PER_SEC / MS_PER_SEC)
#define NS_PER_MS   INT64_C(1000000)    // (NS_PER_SEC / MS_PER_SEC)
#define NS_PER_US   INT64_C(1000)       // (NS_PER_SEC / US_PER_SEC)
// extern void timespec_normalize(struct timespec *ts);
// Convert milliseconds to timespec
// extern void timespec_from_millisec(struct timespec *ts, int64_t ms);
// Convert timespec to milliseconds
// extern int64_t timespec_to_millisec(struct timespec *ts);
// Returns the current (since power-on) timespec |CLOCK_MONOTONIC|, eg. tick / freq
// extern int timespec_get_systime(struct timespec *ts);
// Returns the UTC timespec |CLOCK_REALTIME|, us since 1970/1/1 00:00:00, eg. rtc
// extern int timespec_get_utctime(struct timespec *ts);
// Returns the NTP timespec, us since 1900/1/1 00:00:00, eg. ntp server
// extern int timespec_get_ntptime(struct timespec *ts);

#if defined(_TIME_C_USE_STD)
    // #include <time.h>
#else
    // extern int timespec_get(struct timespec *ts, int base);
#endif /* defined(_TIME_C_USE_STD) */

#if defined(__cplusplus)
}
#endif

#include "time_c_inl.h"

#endif /* _TIME_C_H */