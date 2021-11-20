
// c11: stdatomic
// https://en.cppreference.com/w/c/atomic
// https://gitlab.inria.fr/gustedt/stdatomic
// https://github.com/cdschreiber/c11

#ifndef _ATOMIC_C_H
#define _ATOMIC_C_H

#include "features_config.h"

#if !defined(_ATOMIC_C_USE_STD) \
    && !defined(_ATOMIC_C_USE_POSIX) \
    && !defined(_ATOMIC_C_USE_WIN) \
    && !defined(_ATOMIC_C_USE_NONE)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L) \
        && !defined(__STDC_NO_ATOMICS__)
        #define _ATOMIC_C_USE_STD
    #elif defined(__GNUC__) || defined(__clang__)
        #define _ATOMIC_C_USE_POSIX
    #elif defined(_MSC_VER)
        #define _ATOMIC_C_USE_WIN
    #else
        #define _ATOMIC_C_USE_NONE
    #endif /* !defined(__STDC_NO_ATOMICS__) */
#endif /* !defined(_ATOMIC_C_USE_STD) && ... */

// before include header files
#if defined(_ATOMIC_C_USE_POSIX)
    // #ifndef _POSIX_C_SOURCE // error occurs if < 200809L
    #undef _POSIX_C_SOURCE
    #define _POSIX_C_SOURCE 200809L
    // #endif /* _POSIX_C_SOURCE */
#endif /* defined(_ATOMIC_C_USE_POSIX) */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_ATOMIC_C_USE_STD)
    #include <stdatomic.h>
    // var
    #define ATOMIC_VAR_CAS(_ptr, _oval, _nval) __extension__({ __typeof__(_oval) __tmp = _oval; atomic_compare_exchange_strong((_ptr), &__tmp, _nval); })
    #define ATOMIC_VAR_FAA(_ptr, _val) atomic_fetch_add((_ptr), (_val))
    #define ATOMIC_VAR_XCHG(_ptr, _val) atomic_exchange((_ptr), (_val))
    #define ATOMIC_VAR_STOR(_ptr, _val) atomic_store((_ptr), (_val))
    #define ATOMIC_VAR_LOAD(_ptr) atomic_load(_ptr)
    // flag
    // #define ATOMIC_FLAG_TAS(_ptr) atomic_flag_test_and_set(_ptr) // atomic_exchange((_ptr), true)
    // #define ATOMIC_FLAG_CLR(_ptr) atomic_flag_clear(_ptr) // atomic_exchange((_ptr), false)
#else
    // var
    #define _Atomic(_T) _T
    // #ifndef ATOMIC_VAR_INIT // cannot use others
    #undef ATOMIC_VAR_INIT
    #define ATOMIC_VAR_INIT(_val) (_val)
    // #endif /* ATOMIC_VAR_INIT */
    // flag
    typedef _Atomic(bool) atomic_flag; // typedef atomic_bool atomic_flag;
    #undef ATOMIC_FLAG_INIT
    #define ATOMIC_FLAG_INIT (false)
    #if defined(_ATOMIC_C_USE_POSIX)
        #if defined(__clang__)
            // var
            #define ATOMIC_VAR_CAS(_ptr, _oval, _nval) __c11_atomic_compare_exchange_strong((_ptr), (_oval), (_nval), memory_order_seq_cst, memory_order_seq_cst)
            #define ATOMIC_VAR_FAA(_ptr, _val) __c11_atomic_fetch_add((_ptr), (_val), memory_order_seq_cst)
            #define ATOMIC_VAR_XCHG(_ptr, _val) __c11_atomic_flag_test_and_set((_ptr), (_val))
            #define ATOMIC_VAR_STOR(_ptr, _val) __c11_atomic_store((_ptr), (_val), memory_order_seq_cst)
            #define ATOMIC_VAR_LOAD(_ptr) __c11_atomic_load((_ptr), memory_order_seq_cst)
            // flag
            // #define ATOMIC_FLAG_TAS(_ptr) __c11_atomic_flag_test_and_set(_ptr)
            // #define ATOMIC_FLAG_CLR(_ptr) __c11_atomic_flag_clear(_ptr)
        #elif defined(__GNUC__)
            // use __sync or __atomic
            // var
            #define ATOMIC_VAR_CAS(_ptr, _oval, _nval) __sync_bool_compare_and_swap((_ptr), (_oval), (_nval))
            #define ATOMIC_VAR_FAA(_ptr, _val) __sync_fetch_and_add((_ptr), (_val))
            #define ATOMIC_VAR_XCHG(_ptr, _val) __sync_lock_test_and_set((_ptr), (_val))
            #define ATOMIC_VAR_STOR(_ptr, _val) (void)(*(_ptr) = (_val))
            #define ATOMIC_VAR_LOAD(_ptr) (*(_ptr))
            // flag
            // #define ATOMIC_FLAG_TAS(_ptr) __sync_lock_test_and_set(_ptr, true)
            // #define ATOMIC_FLAG_CLR(_ptr) __sync_lock_release(_ptr)
        #else
            #error "atomic_c: unknown branch in Posix |ATOMIC_VAR_XXX|"
        #endif /* defined(__clang__) */
    #elif defined(_ATOMIC_C_USE_WIN)
        // Include winsock2.h before including <windows.h> to maintain consistency with
        // win32.h. To include win32.h directly, it must be broken out into its own
        // build target.
        #include <winsock2.h>
        #include <windows.h>
        // var
        #define ATOMIC_VAR_CAS(_ptr, _oval, _nval) (_InterlockedCompareExchange((_ptr), (_nval), (_oval)) == (_oval))
        #define ATOMIC_VAR_FAA(_ptr, _val) _InterlockedExchangeAdd((_ptr), (_val))
        #define ATOMIC_VAR_XCHG(_ptr, _val) _InterlockedExchange((_ptr), (_val))
        #define ATOMIC_VAR_STOR(_ptr, _val) (void)(*(_ptr) = (_val))
        #define ATOMIC_VAR_LOAD(_ptr) (*(_ptr))
        // flag
        // #define ATOMIC_FLAG_TAS(_ptr) _InterlockedExchange(_ptr, true)
        // #define ATOMIC_FLAG_CLR(_ptr) _InterlockedExchange(_ptr, false)
    #elif defined(_ATOMIC_C_USE_NONE)
        // var
        #define ATOMIC_VAR_CAS(_ptr, _oval, _nval) __extension__({ bool __ret; if ((uintptr_t)_oval == *(uintptr_t *)(_ptr)) { *(_ptr) = (_nval); __ret = true; } else { __ret = false; } __ret; })
        #define ATOMIC_VAR_FAA(_ptr, _val) __extension__({ __typeof__(*(_ptr)) __oval = *(_ptr); *(_ptr) += (_val); __oval; })
        #define ATOMIC_VAR_XCHG(_ptr, _val) __extension__({ __typeof__(*(_ptr)) __oval = *(_ptr); *(_ptr) = (_val); __oval; })
        #define ATOMIC_VAR_STOR(_ptr, _val) (void)(*(_ptr) = (_val))
        #define ATOMIC_VAR_LOAD(_ptr) (*(_ptr))
        // flag
        // #define ATOMIC_FLAG_TAS(_ptr) __extension__({ bool __oval = *(_ptr); if (!__oval) { *(_ptr) = true; } __oval; })
        // #define ATOMIC_FLAG_CLR(_ptr) (void)(*(_ptr) = false)
    #else
        #error "atomic_c: unknown branch in |ATOMIC_VAR_XXX|"
    #endif /* defined(_ATOMIC_C_USE_POSIX) */
#endif /* defined(_ATOMIC_C_USE_STD) */
// ATOMIC_VAR should be smaller than WORDSIZE(maybe (u)intptr_t)
#define ATOMIC_VAR(_type) _Atomic(_type)
// ATOMIC_FLAG always uses with |volatile|
#define ATOMIC_FLAG volatile atomic_flag
// alway use xchg for flag
#define ATOMIC_FLAG_TAS(_ptr) ATOMIC_VAR_XCHG((_ptr), true)
#define ATOMIC_FLAG_CLR(_ptr) (void)ATOMIC_VAR_XCHG((_ptr), false)

#if defined(__cplusplus)
}
#endif

#endif /* _ATOMIC_C_H */
