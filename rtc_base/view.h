
#ifndef RTC_BASE_VIEW_H_
#define RTC_BASE_VIEW_H_

#include <stddef.h>
#include "rtc_base/checks.h"
#include "rtc_base/constructor_magic.h"

#if defined(__cplusplus)
// C++ version.
#include <algorithm>
// #include <array>

namespace rtc {

template <typename T>
struct View {
 public:
  // Construct View from a pointer and a length.
  template <typename U>
  View(U* data, size_t size)
      : data_(data),
        size_(size) {
    RTC_DCHECK_EQ(size == 0 ? nullptr : data, this->data());
    RTC_DCHECK_EQ(size, this->size());
    RTC_DCHECK_EQ(!this->data(),
                  this->size() == 0);  // data is null if size == 0.
  }
  // Construct an empty View. Note that fixed-size Views of size > 0
  // cannot be empty.
  View() 
    : View(static_cast<T*>(nullptr), 0) {}
  /* View(std::nullptr_t)  // NOLINT
      : View() {}
  View(std::nullptr_t, size_t size)
      : View(static_cast<T*>(nullptr), size) {
    RTC_DCHECK_EQ(0, size);
  } */
  // Construct an View from a C-style array.
  template <typename U, size_t N>
  View(U (&array)[N])  // NOLINT
      : View(array, N) {}
  // Construct an View from any type U that
  // has a size() method whose return value converts implicitly to size_t, and
  // a data() method whose return value converts implicitly to T*.
  // std::array and rtc::View itself can also be constructed by these method
  template <typename U>
  View(U& u)  // NOLINT
      : View(u.data(), u.size()) {}
  template <typename U>
  View(const U& u)  // NOLINT(runtime/explicit)
      : View(u.data(), u.size()) {}

  T* data() const { return data_; }
  size_t size() const { return size_; }
  bool empty() const { return this->size() == 0; }
  // Indexing and iteration. These allow mutation even if the View is
  // const, because the View doesn't own the array. (To prevent mutation,
  // use a const element type.)
  T& operator[](size_t idx) const {
    RTC_DCHECK_LT(idx, this->size());
    RTC_DCHECK(this->data());
    return this->data()[idx];
  }
  T* begin() const { return this->data(); }
  T* end() const { return this->data() + this->size(); }
  /* const T* cbegin() const { return this->data(); }
  const T* cend() const { return this->data() + this->size(); } */
  T& front() const { return *this->begin(); } // *this->begin()
  T& back() const { return *std::prev(this->end()); } // *(--this->end())
  void fill(T value) { std::fill(this->begin(), this->end(), value); }

  View<T> subview(size_t offset, size_t size) const {
    return offset < this->size()
               ? View<T>(this->data() + offset,
                         std::min(size, this->size() - offset))
               : View<T>();
  }
  View<T> subview(size_t offset) const {
    return subview(offset, this->size());
  }
  // Comparing two Views compares their (pointer,size) pairs; it does *not*
  // dereference the pointers.
  /* template <typename T>
  bool operator==(const View<T>& a, const View<T>& b) {
    return a.data() == b.data() && a.size() == b.size();
  }
  template <typename T>
  bool operator!=(const View<T>& a, const View<T>& b) {
    return !(a == b);
  } */

 private:
  T* data_;
  size_t size_;

  // RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(View);
};

/* template <typename T>
inline View<T> MakeView(T* data, size_t size) {
  return View<T>(data, size);
} */

} // namespace rtc

#define RTC_VIEW(T) rtc::View<T>
#define RTC_MAKE_VIEW(T) rtc::View<T> // rtc::MakeView<T>


#if 0 /* container */

namespace rtc {

template <typename T>
struct Container {
//  public:
//   Container(T data) : data_(data) {};
//   T& data() { return data_; }

//  private:
  T data_;
};

} // namespace rtc

#define RTC_CONTAINER(T) rtc::Container<T>

#else /* container */

#include <array>

/* namespace rtc {

template <typename T>
struct Array {
 // public:
  // Construct an View from a C-style array.
  template <typename U, size_t N>
  Array(U (&data)[N])  // NOLINT
      : data_(data),
        view_(RTC_MAKE_VIEW(U)(data_)) {};
  // cannot things below, eg.
  //    T is float[96], we need view_ to be
  //    RTC_VIEW(float) not RTC_VIEW(float[96]) 
  // T* data() { return view_.data(); }
  // size_t size() const { return view_.size(); }
  // T& operator[](size_t idx) const { return view_[idx]; }
  // T* begin() const { return view_.begin(); }
  // T* end() const { return view_.end(); }
  // void fill(T value) { view_.fill(value); }

 // private:
  T data_;
  // RTC_VIEW(T) view_; // cannot get U
};

} // namespace rtc

#define RTC_ARRAY(T) rtc::array<T> */

#endif /* container */

// From rtc_base/arraysize.h
#include <stddef.h>

// This file defines the arraysize() macro and is derived from Chromium's
// base/macros.h.

// The arraysize(arr) macro returns the # of elements in an array arr.
// The expression is a compile-time constant, and therefore can be
// used in defining new arrays, for example.  If you use arraysize on
// a pointer by mistake, you will get a compile-time error.

// This template function declaration is used in defining arraysize.
// Note that the function doesn't need an implementation, as we only
// use its type.
template <typename T, size_t N>
char (&ArraySizeHelper(T (&array)[N]))[N];

#define arraysize(array) (sizeof(ArraySizeHelper(array)))

#elif 0 // #else  // __cplusplus not defined

// C version. Lacks many features compared to the C++ version, but usage
// guidelines are the same.

typedef struct { void* data_; size_t size_; } rtc_View;
inline rtc_View rtc_MakeView(void *data, size_t size)
{
    return (rtc_View) { data, size, };
}

// error: types may not be defined in parameter types
#define RTC_VIEW(T) rtc_View // struct { T* data_; size_t size_; }
#define RTC_MAKE_VIEW(T) rtc_MakeView

// RTC_VIEW(float) xxx_view = RTC_MAKE_VIEW(float)(xxx.data(), xxx.size());

// #define RTC_CONTAINER(T) struct { T data_; }

#define arraysize(array) (sizeof(array)/sizeof(array[0]))

#endif  // defined(__cplusplus)

#endif  // RTC_BASE_VIEW_H_
