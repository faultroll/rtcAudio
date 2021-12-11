/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains platform-specific typedefs and defines.
// Much of it is derived from Chromium's build/build_config.h.

#ifndef RTC_BASE_SYSTEM_COMPILE_MAGIC_H_
#define RTC_BASE_SYSTEM_COMPILE_MAGIC_H_

// From rtc_base/checks.h
#ifndef RTC_NORETURN
// Annotate a function that will not return control flow to the caller.
#if defined(WEBRTC_WIN)
#define RTC_NORETURN __declspec(noreturn)
#elif defined(WEBRTC_POSIX)
#define RTC_NORETURN __attribute__ ((__noreturn__))
#else
#define RTC_NORETURN
#endif // defined(WEBRTC_WIN)
#endif // RTC_NORETURN

// Added, used in rtc_base/critical_section.h
#ifndef RTC_CHECKRETURN
#if defined(WEBRTC_WIN)
#define RTC_CHECKRETURN _Check_return_
#elif defined(WEBRTC_POSIX)
#define RTC_CHECKRETURN __attribute__ ((__warn_unused_result__))
#else
#define RTC_CHECKRETURN
#endif // defined(WEBRTC_WIN)
#endif // RTC_CHECKRETURN

// From rtc_base/system/unused.h
// Annotate a function indicating the caller must examine the return value.
// Use like:
//   int foo() RTC_WARN_UNUSED_RESULT;
// To explicitly ignore a result, cast to void.
// TODO(kwiberg): Remove when we can use [[nodiscard]] from C++17.
#if defined(__clang__)
#define RTC_WARN_UNUSED_RESULT __attribute__((__warn_unused_result__))
#elif defined(__GNUC__)
// gcc has a __warn_unused_result__ attribute, but you can't quiet it by
// casting to void, so we don't use it.
#define RTC_WARN_UNUSED_RESULT
#else
#define RTC_WARN_UNUSED_RESULT
#endif

// From rtc_base/system/unused.h
// Prevent the compiler from warning about an unused variable. For example:
//   int result = DoSomething();
//   assert(result == 17);
//   RTC_UNUSED(result);
// Note: In most cases it is better to remove the unused variable rather than
// suppressing the compiler warning.
#ifndef RTC_UNUSED
#define RTC_UNUSED(x) static_cast<void>(x)
#endif // RTC_UNUSED

// From rtc_base/system/rtc_export.h
// RTC_EXPORT is used to mark symbols as exported or imported when WebRTC is
// built or used as a shared library.
// When WebRTC is built as a static library the RTC_EXPORT macro expands to
// nothing.
#ifndef RTC_EXPORT
#if defined(WEBRTC_WIN)
#define RTC_EXPORT __declspec(dllexport)
#elif defined(WEBRTC_POSIX)
#define RTC_EXPORT __attribute__((visibility("default")))
#else
#define RTC_EXPORT
#endif // defined(WEBRTC_WIN)
#endif // RTC_EXPORT

// From rtc_base/system/ignore_warnings.h
#ifdef __clang__
#define RTC_PUSH_IGNORING_WFRAME_LARGER_THAN() \
  _Pragma("clang diagnostic push")             \
      _Pragma("clang diagnostic ignored \"-Wframe-larger-than=\"")
#define RTC_POP_IGNORING_WFRAME_LARGER_THAN() _Pragma("clang diagnostic pop")
#elif __GNUC__
#define RTC_PUSH_IGNORING_WFRAME_LARGER_THAN() \
  _Pragma("GCC diagnostic push")               \
      _Pragma("GCC diagnostic ignored \"-Wframe-larger-than=\"")
#define RTC_POP_IGNORING_WFRAME_LARGER_THAN() _Pragma("GCC diagnostic pop")
#else
#define RTC_PUSH_IGNORING_WFRAME_LARGER_THAN()
#define RTC_POP_IGNORING_WFRAME_LARGER_THAN()
#endif

// From rtc_base/deprecation.h
// Annotate the declarations of deprecated functions with this to cause a
// compiler warning when they're used. Like so:
//
//   RTC_DEPRECATED std::pony PonyPlz(const std::pony_spec& ps);
//
// NOTE 1: The annotation goes on the declaration in the .h file, not the
// definition in the .cc file!
//
// NOTE 2: In order to keep unit testing the deprecated function without
// getting warnings, do something like this:
//
//   std::pony DEPRECATED_PonyPlz(const std::pony_spec& ps);
//   RTC_DEPRECATED inline std::pony PonyPlz(const std::pony_spec& ps) {
//     return DEPRECATED_PonyPlz(ps);
//   }
//
// In other words, rename the existing function, and provide an inline wrapper
// using the original name that calls it. That way, callers who are willing to
// call it using the DEPRECATED_-prefixed name don't get the warning.
//
// TODO(kwiberg): Remove this when we can use [[deprecated]] from C++14.
#if defined(_MSC_VER)
// Note: Deprecation warnings seem to fail to trigger on Windows
// (https://bugs.chromium.org/p/webrtc/issues/detail?id=5368).
#define RTC_DEPRECATED __declspec(deprecated)
#elif defined(__GNUC__)
#define RTC_DEPRECATED __attribute__((__deprecated__))
#else
#define RTC_DEPRECATED
#endif

// From rtc_base/compile_assert_c.h
// Use this macro to verify at compile time that certain restrictions are met.
// The argument is the boolean expression to evaluate.
// Example:
//   RTC_COMPILE_ASSERT(sizeof(foo) < 128);
// Note: In C++, use static_assert instead!
#define RTC_COMPILE_ASSERT(expression) \
  switch (0) {                         \
    case 0:                            \
    case expression:;                  \
  }
#ifndef __cplusplus
#define static_assert(expression, message) RTC_COMPILE_ASSERT(expression)
#endif

#endif  // RTC_BASE_SYSTEM_COMPILE_MAGIC_H_
