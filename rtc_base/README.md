
## basic:
- features // `<catomic.h> <cthread.h> <ctime.h> for c11 features, optional for c++14/17 features`
- atomic_ops.h // `c11 <stdatomic.h> with other funcs`
- checks.cc
- checks.h // check macros
- constructor_magic.h // class macros
- critical_section.cc
- critical_section.h // mutex locks
- platform_thread_types.cc
- platform_thread_types.h // `c11 <threads.h> with other funcs`
- system // compile macros
- thread_annotations.h // thread macros
- time_utils.cc
- time_utils.h // `c11 <time.h> timespec and millisec`
- aligned_malloc.h // `c11 <stdlib.h> aligned_malloc`

## rtcThread:
- event.cc
- event.h // semaphore
- location.cc 
- location.h // where message froms
- message_handler.cc
- message_handler.h // message handler interface
- null_socket_server.cc
- null_socket_server.h // default socket server for rtc::thread
- platform_thread.cc
- platform_thread.h // thread implement for rtc::thread
- race_checker.cc
- race_checker.h // race checker
- socket_factory.h // socket server
- socket_server.h // socket server
- synchronization // sequence checker
- thread.cc
- thread.h // rtc::thread
- thread_message.h // message interface

## rtcAudio:
- arraysize.h
- numerics // safe_xxx conversion/min_max/...
- scoped_refptr.h // reference pointer, in api ver. m88
- ref_counted_object.h // same? as scoped_refptr? 
- ref_counter.h
- ref_count.h
- swap_queue // for signal processing
- asm_defines.h // asm utils for signal processing
- arch.h // basictypes.h
### rtcAudio from system_wrappers:
- metrics.h // metric for histogram

## rtcRtp:
- arraysize.h
- numerics
- scoped_refptr.h
- random.h // random
- buffer.h // buffers, move out of rtc_base 
- bit_buffer.cc
- bit_buffer.h
- // byte_buffer.cc
- // byte_buffer.h
- rate_statistics.cc // rate control, move out of rtc_base
- rate_statistics.h
- rate_limiter.cc
- rate_limiter.h
### rtcRtp from system_wrappers:
- ntp_time.h // timestamp utils
- clock.h
- timestamp_extrapolator.h
