
#include "time_c.h"

void timespec_normalize(struct timespec *ts)
{
    if (ts->tv_nsec >= NS_PER_SEC) {
        ts->tv_sec  += ts->tv_nsec / NS_PER_SEC;
        ts->tv_nsec = ts->tv_nsec % NS_PER_SEC;
        /* ts->tv_sec++;
        ts->tv_nsec -= NS_PER_SEC; */
    }
}
void timespec_from_millisec(struct timespec *ts, int64_t ms)
{
    ts->tv_sec  = ms / MS_PER_SEC;
    ts->tv_nsec = (ms % MS_PER_SEC) * NS_PER_MS;
    timespec_normalize(ts);
}
int64_t timespec_to_millisec(struct timespec *ts)
{
    return ((int64_t)(ts->tv_sec) * MS_PER_SEC +
            (int64_t)(ts->tv_nsec) / NS_PER_MS);
}

#if defined(_TIME_C_USE_STD)
    int timespec_get_systime(struct timespec *ts)
    {
        // how to get systick?
        return timespec_get(ts, TIME_UTC);
    }
    int timespec_get_utctime(struct timespec *ts)
    {
        return timespec_get(ts, TIME_UTC);
    }
    // int timespec_get(struct timespec *ts, int base);
#else

    #if defined(_TIME_C_USE_POSIX)
        int timespec_get_systime(struct timespec *ts)
        {
            // TODO(deadbeef): Do we need to handle the case when CLOCK_MONOTONIC is not
            // supported?
            return clock_gettime(CLOCK_MONOTONIC, ts);
        }
        int timespec_get_utctime(struct timespec *ts)
        {
        #if 0
            struct timeval time;
            gettimeofday(&time, nullptr);
            // Convert from second (1.0) and microsecond (1e-6).
            return ((int64_t)(time.tv_sec) * MS_PER_SEC +
                    (int64_t)(time.tv_usec) / US_PER_MS);
        #else
            return clock_gettime(CLOCK_REALTIME, ts);
        #endif
        }
    #elif defined(_TIME_C_USE_WIN)
        int timespec_get_systime(struct timespec *ts)
        {
            int64_t ticks;

            static volatile LONG last_timegettime = 0;
            static volatile int64_t num_wrap_timegettime = 0;
            volatile LONG *last_timegettime_ptr = &last_timegettime;
            DWORD now = timeGetTime();
            // SYSTEMTIME system_time;
            // GetSystemTime(&system_time);
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
            // ticks = ticks * NS_PER_MS;

            timespec_from_millisec(ts, ticks);

            return 0;
        }
        int timespec_get_utctime(struct timespec *ts)
        {
            struct _timeb time;
            _ftime(&time);
            // Convert from second (1.0) and milliseconds (1e-3).
            ts->tv_sec = (int64_t)(time.time);
            ts->tv_nsec = (int64_t)(time.millitm) * NS_PER_MS;

            return 0;
        }
    #elif defined(_TIME_C_USE_NONE)
        /* int timespec_get_systime(struct timespec *ts)
        {
            int64_t ticks;
            // how to get systick?
            ticks = systick / freq;
            timespec_from_millisec(ts, ticks);

            return 0;
        }
        int timespec_get_utctime(struct timespec *ts)
        {
            // how to get time from rtc?
            return timespec_get_systime(ts);
        } */
        #error "time_c: current not support |_TIME_C_USE_NONE|"
    #else
        #error "time_c: unknown branch in |timespec_get_xxx|"
    #endif /* defined(_TIME_C_USE_POSIX) */
    int timespec_get(struct timespec *ts, int base)
    {
        return timespec_get_utctime(ts);
    }
#endif /* defined(_TIME_C_USE_STD) */

int timespec_get_ntptime(struct timespec *ts)
{
    // http://doc.ntp.org/archives/4.1.2/leap/
    static const int64_t kNtpJan1970Sec = INT64_C(2208988800);
    timespec_get_utctime(ts);
    ts->tv_sec += kNtpJan1970Sec;

    return 0;
}
