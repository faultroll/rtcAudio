# in use
srcs_c += \
        rtc_base/features/align_c.c \
        rtc_base/features/time_c.c \
        rtc_base/features/thread_c.c \
        rtc_base/checks_c.c
srcs += \
        rtc_base/features/optional.cc \
        rtc_base/checks.cc \
        rtc_base/time_utils.cc \
        rtc_base/platform_thread_types.cc \
        rtc_base/critical_section.cc \
        rtc_base/memory/aligned_malloc.cc \
        rtc_base/trace_impl.cc
# just compile, currently not in use
srcs_cpl_only += \
        rtc_base/platform_thread.cc \
        rtc_base/event.cc \
        rtc_base/location.cc \
        rtc_base/message_handler.cc \
        rtc_base/null_socket_server.cc \
        rtc_base/thread.cc \
        rtc_base/synchronization/sequence_checker.cc \
        rtc_base/race_checker.cc \
        rtc_base/bit_buffer.cc \
        rtc_base/rate_statistics.cc \
        rtc_base/random.cc
