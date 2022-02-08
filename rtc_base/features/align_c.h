
// c11: stdalign aligned_alloc
// https://github.com/cdschreiber/c11
// https://github.com/open-webrtc-toolkit/owt-deps-webrtc

#ifndef _ALIGN_C_H
#define _ALIGN_C_H

#include "features_config.h"

#if !defined(_ALIGN_C_USE_STD) \
    && !defined(_ALIGN_C_USE_POSIX) \
    && !defined(_ALIGN_C_USE_WIN) \
    && !defined(_ALIGN_C_USE_NONE)
    #if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
        #define _ALIGN_C_USE_STD
    #elif defined(__GNUC__) || defined(__clang__)
        #define _ALIGN_C_USE_POSIX
    #elif defined(_MSC_VER)
        #define _ALIGN_C_USE_WIN
    #else
        #define _ALIGN_C_USE_NONE
    #endif /* !defined(__STDC_NO_ATOMICS__) */
#endif /* !defined(_ALIGN_C_USE_STD) && ... */

// before include header files
#if defined(_ALIGN_C_USE_POSIX)
    #undef _POSIX_C_SOURCE
    #define _POSIX_C_SOURCE 200809L
#endif /* defined(_ALIGN_C_USE_POSIX) */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_ALIGN_C_USE_STD)
    #include <stdalign.h> // for alignas, alignof
    #include <stdlib.h> // for aligned_alloc, free
    #define aligned_free(mem_block) free(mem_block)
#else
    #if defined(_ALIGN_C_USE_POSIX)
        #ifndef __alignas_is_defined
            #define alignas(size) __attribute__((__aligned__(size)))
            #define __alignas_is_defined 1
        #endif /* __alignas_is_defined */
        #ifndef __alignof_is_defined
            #define alignof(type) __alignof__(type)
            #define __alignof_is_defined 1
        #endif /* __alignof_is_defined */
    #elif defined(_ALIGN_C_USE_WIN)
        #ifndef __alignas_is_defined
            #define alignas(size) __declspec(align(size))
            #define __alignas_is_defined 1
        #endif /* __alignas_is_defined */
        #ifndef __alignof_is_defined
            #define alignof(type) __alignof(type)
            #define __alignof_is_defined 1
        #endif /* __alignof_is_defined */
    #elif defined(_ALIGN_C_USE_NONE)
        #ifndef __alignas_is_defined
            #define alignas(size)
            #define __alignas_is_defined 1
        #endif /* __alignas_is_defined */
        #ifndef __alignof_is_defined
            #define alignof(type) sizeof(type)
            #define __alignof_is_defined 1
        #endif /* __alignof_is_defined */
    #else
        #error "align_c: unknown branch in |alignxxx|"
    #endif /* defined(_ALIGN_C_USE_POSIX) */
    // extern void* aligned_alloc(size_t size, size_t alignment);
    // extern void aligned_free(void* mem_block);
#endif /* defined(_ALIGN_C_USE_STD) */
// Alignment must be an integer power of two.
#define ALIGN_VALID(a) (((a)!=0)&&(0==((a)&((a)-1))))
#define ALIGN_UP(x, a) (((x)+(a)-1)&~((a)-1)) // ((a)*(((x)+(a)-1)/(a)))
#define ALIGN_DOWN(x, a) ((x)&~((a)-1)) // ((a)*((x)/(a))) 

#if defined(__cplusplus)
}
#endif

// Work as inline function (c file)
// TODO(lgY): split it into different files (posix/win/none)
#include "align_c_inl.h"

#endif /* _ALIGN_C_H */
