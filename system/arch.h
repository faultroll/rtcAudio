/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains platform-specific typedefs and defines.
// Much of it is derived from Chromium's build/build_config.h.

#ifndef RTC_BASE_SYSTEM_ARCH_H_
#define RTC_BASE_SYSTEM_ARCH_H_

// Processor architecture detection.  For more info on what's defined, see:
//   http://msdn.microsoft.com/en-us/library/b0084kay.aspx
//   http://www.agner.org/optimize/calling_conventions.pdf
//   or with gcc, run: "echo | gcc -E -dM -"
#if defined(_M_X64) || defined(__x86_64__) || defined(_M_AMD64)
#define WEBRTC_ARCH_X86_FAMILY
#define WEBRTC_ARCH_X86_64
#define WEBRTC_ARCH_64_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(_M_ARM64) || defined(__aarch64__)
#define WEBRTC_ARCH_ARM_FAMILY
#define WEBRTC_ARCH_64_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__riscv) || defined(__riscv__)
#if __riscv_xlen == 64
#define WEBRTC_ARCH_64_BITS
#else
#define WEBRTC_ARCH_32_BITS
#endif
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(_M_IX86) || defined(__i386__)
#define WEBRTC_ARCH_X86_FAMILY
#define WEBRTC_ARCH_X86
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__ARMEL__)
#define WEBRTC_ARCH_ARM_FAMILY
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__MIPSEL__)
#define WEBRTC_ARCH_MIPS_FAMILY
#if defined(__LP64__)
#define WEBRTC_ARCH_64_BITS
#else
#define WEBRTC_ARCH_32_BITS
#endif
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__pnacl__)
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#elif defined(__EMSCRIPTEN__)
#define WEBRTC_ARCH_32_BITS
#define WEBRTC_ARCH_LITTLE_ENDIAN
#else
#error Please add support for your architecture in rtc_base/system/arch.h
#endif

#if !(defined(WEBRTC_ARCH_LITTLE_ENDIAN) ^ defined(WEBRTC_ARCH_BIG_ENDIAN))
#error Define either WEBRTC_ARCH_LITTLE_ENDIAN or WEBRTC_ARCH_BIG_ENDIAN
#endif

/**
 * WEBRTC_HAS_XXX
 *  WEBRTC_ARCH_X86_FAMILY: SSE2, SSE3, AVX2
 *  WEBRTC_ARCH_ARM_FAMILY: ARM_NEON
 *  WEBRTC_ARCH_MIPS_FAMILY: MIPS_FPU, MIPS_DSP
 *
 **/
#if !defined(WEBRTC_HAS_AVX2) && !defined(WEBRTC_HAS_SSE2) \
    && !defined(WEBRTC_HAS_NEON) \
    /* && !defined(WEBRTC_HAS_MIPS_FPU) */
    #if defined(WEBRTC_ARCH_X86_FAMILY)
        #if defined(__AVX2__)
            #define WEBRTC_HAS_AVX2
        // #elif defined(__AVX__)
        #elif defined(WEBRTC_ARCH_64_BITS)
            #define WEBRTC_HAS_SSE2 // 64bit
        #elif _M_IX86_FP == 2
            #define WEBRTC_HAS_SSE2 // 32bit
        // #elif _M_IX86_FP == 1
        //     #define WEBRTC_HAS_SSE // 32bit
        #endif
    /**
      * if (current_cpu == "arm64") {
      *   defines += [ "WEBRTC_ARCH_ARM64" ]
      *   defines += [ "WEBRTC_HAS_NEON" ]
      * }
      * if (current_cpu == "arm") {
      *   defines += [ "WEBRTC_ARCH_ARM" ]
      *   if (arm_version >= 7) {
      *     defines += [ "WEBRTC_ARCH_ARM_V7" ]
      *     if (arm_use_neon) {
      *       defines += [ "WEBRTC_HAS_NEON" ]
      *     }
      *   }
      * }
      *
      */
    #elif defined(WEBRTC_ARCH_ARM_FAMILY)
        #if defined(__ARM_NEON) || defined(__ARM_NEON__)
            #define WEBRTC_HAS_NEON
        #endif
        #if (__ARM_ARCH >= 7) && (defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7A))
            #define WEBRTC_ARCH_ARM_V7
        #elif (__ARM_ARCH == 5)
            #define WEBRTC_ARCH_ARM_V5
        #endif
    /**
      * if (current_cpu == "mipsel") {
      *   defines += [ "MIPS32_LE" ]
      *   if (mips_float_abi == "hard") {
      *     defines += [ "MIPS_FPU_LE" ]
      *   }
      *   if (mips_arch_variant == "r2") {
      *     defines += [ "MIPS32_R2_LE" ]
      *   }
      *   if (mips_dsp_rev == 1) {
      *     defines += [ "MIPS_DSP_R1_LE" ]
      *   } else if (mips_dsp_rev == 2) {
      *     defines += [
      *       "MIPS_DSP_R1_LE",
      *       "MIPS_DSP_R2_LE",
      *     ]
      *   }
      * }
      *
      */
    /* #elif defined(WEBRTC_ARCH_MIPS_FAMILY)
        #if defined(MIPS_FPU_LE)
            #define WEBRTC_HAS_MIPS_FPU
        #endif
        #if defined(MIPS_DSP_R1_LE)
            #define WEBRTC_HAS_MIPS_DSP 1
        #elif defined(MIPS_DSP_R2_LE)
            #define WEBRTC_HAS_MIPS_DSP 2
        #endif */
    #endif
#endif

#endif  // RTC_BASE_SYSTEM_ARCH_H_
