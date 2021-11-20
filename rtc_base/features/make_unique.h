
// cxx14: std::make_unique

#ifndef _MAKE_UNIQUE_H
#define _MAKE_UNIQUE_H

#include <memory> // std::unique_ptr

namespace rtc {

// gcc 4.8 has __cplusplus at 201301 but the libstdc++ shipped with it doesn't
// define make_unique.  Other supported compilers either just define __cplusplus
// as 201103 but have make_unique (msvc), or have make_unique whenever
// __cplusplus > 201103 (clang).
#if (__cplusplus > 201103L || defined(_MSC_VER)) && \
    !(defined(__GLIBCXX__) && !defined(__cpp_lib_make_unique))

using std::make_unique;

#else /* (__cplusplus >= 201103L) ... */

// -----------------------------------------------------------------------------
// Function Template: make_unique<T>()
// -----------------------------------------------------------------------------
//
// Creates a `std::unique_ptr<>`, while avoiding issues creating temporaries
// during the construction process. `absl::make_unique<>` also avoids redundant
// type declarations, by avoiding the need to explicitly use the `new` operator.
//
// This implementation of `absl::make_unique<>` is designed for C++11 code and
// will be replaced in C++14 by the equivalent `std::make_unique<>` abstraction.
// `absl::make_unique<>` is designed to be 100% compatible with
// `std::make_unique<>` so that the eventual migration will involve a simple
// rename operation.
//
// For more background on why `std::unique_ptr<T>(new T(a,b))` is problematic,
// see Herb Sutter's explanation on
// (Exception-Safe Function Calls)[https://herbsutter.com/gotw/_102/].
// (In general, reviewers should treat `new T(a,b)` with scrutiny.)
//
// Example usage:
//
//    auto p = make_unique<X>(args...);  // 'p'  is a std::unique_ptr<X>
//    auto pa = make_unique<X[]>(5);     // 'pa' is a std::unique_ptr<X[]>
//
// Three overloads of `absl::make_unique` are required:
//
//   - For non-array T:
//
//       Allocates a T with `new T(std::forward<Args> args...)`,
//       forwarding all `args` to T's constructor.
//       Returns a `std::unique_ptr<T>` owning that object.
//
//   - For an array of unknown bounds T[]:
//
//       `absl::make_unique<>` will allocate an array T of type U[] with
//       `new U[n]()` and return a `std::unique_ptr<U[]>` owning that array.
//
//       Note that 'U[n]()' is different from 'U[n]', and elements will be
//       value-initialized. Note as well that `std::unique_ptr` will perform its
//       own destruction of the array elements upon leaving scope, even though
//       the array [] does not have a default destructor.
//
//       NOTE: an array of unknown bounds T[] may still be (and often will be)
//       initialized to have a size, and will still use this overload. E.g:
//
//         auto my_array = absl::make_unique<int[]>(10);
//
//   - For an array of known bounds T[N]:
//
//       `absl::make_unique<>` is deleted (like with `std::make_unique<>`) as
//       this overload is not useful.
//
//       NOTE: an array of known bounds T[N] is not considered a useful
//       construction, and may cause undefined behavior in templates. E.g:
//
//         auto my_array = absl::make_unique<int[10]>();
//
//       In those cases, of course, you can still use the overload above and
//       simply initialize it to its desired size:
//
//         auto my_array = absl::make_unique<int[]>(10);

namespace memory_internal {

// Traits to select proper overload and return type for `absl::make_unique<>`.
template <typename T>
struct MakeUniqueResult {
  using scalar = std::unique_ptr<T>;
};
// template <typename T>
// struct MakeUniqueResult<T[]> {
//   using array = std::unique_ptr<T[]>;
// };
// template <typename T, size_t N>
// struct MakeUniqueResult<T[N]> {
//   using invalid = void;
// };

}  // namespace memory_internal

// `absl::make_unique` overload for non-array types.
//  variadic argument, brace sucks, eg.
//    auto p = make_unique<X>(a, b, c); // OK, three args
//    auto p = make_unique<X>((a, b, c)); // Wrong, only one arg
template <typename T, typename... Args>
typename memory_internal::MakeUniqueResult<T>::scalar make_unique(
    Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
// cannot alias function, so just use make_unique not MakeUnique
// template <typename T, typename... Args>
// auto MakeUnique = make_unique<T, Args ...>;

// `absl::make_unique` overload for an array T[] of unknown bounds.
// The array allocation needs to use the `new T[size]` form and cannot take
// element constructor arguments. The `std::unique_ptr` will manage destructing
// these array elements.
// template <typename T>
// typename memory_internal::MakeUniqueResult<T>::array make_unique(size_t n) {
//   return std::unique_ptr<T>(new typename std::remove_extent<T>::type[n]());
// }

// `absl::make_unique` overload for an array T[N] of known bounds.
// This construction will be rejected.
// template <typename T, typename... Args>
// typename memory_internal::MakeUniqueResult<T>::invalid make_unique(
//     Args&&... /* args */) = delete;

#endif /* (__cplusplus >= 201103L) ... */

}  // namespace rtc


#endif  // _MAKE_UNIQUE_H
