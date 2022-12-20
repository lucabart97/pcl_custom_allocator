#ifndef COMMON_INCLUDE_PCL_TK_ALLOCATOR
#define COMMON_INCLUDE_PCL_TK_ALLOCATOR

#include <Eigen/Core>

#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

namespace tk {

template <class T>
class tk_allocator : public std::allocator<T> {
public:
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T value_type;

  size_type maxSize = 0;
  pointer dataMapped = nullptr;

  template <class U>
  struct rebind {
    typedef tk_allocator<U> other;
  };

  tk_allocator() : std::allocator<T>() {}

  tk_allocator(T* data, size_type maxSize)
  {
    this->dataMapped = data;
    this->maxSize = maxSize;
  }

  tk_allocator(const tk_allocator& other) : std::allocator<T>(other)
  {
    this->dataMapped = other.dataMapped;
    this->maxSize = other.maxSize;
  }

  template <class U>
  tk_allocator(const tk_allocator<U>& other) : std::allocator<T>(other)
  {
    // this->dataMapped = other.dataMapped; // ??
    // this->maxSize = other.maxSize; // ??
  }

  ~tk_allocator() {}

#if EIGEN_COMP_GNUC_STRICT && EIGEN_GNUC_AT_LEAST(7, 0)
  // In gcc std::allocator::max_size() is bugged making gcc triggers a warning:
  // eigen/Eigen/src/Core/util/Memory.h:189:12: warning: argument 1 value
  // '18446744073709551612' exceeds maximum object size 9223372036854775807 See
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=87544
  size_type
  max_size() const
  {
    return (std::numeric_limits<std::ptrdiff_t>::max)() / sizeof(T);
  }
#endif

  pointer
  allocate(size_type num, const void* /*hint*/ = 0)
  {
    if (dataMapped == nullptr) {
      Eigen::internal::check_size_for_overflow<T>(num);
      return static_cast<pointer>(Eigen::internal::aligned_malloc(num * sizeof(T)));
    }
    else {
      assertm(num < maxSize, "allocate error: you request more elements than maxSize");
      return dataMapped;
    }
  }

  void
  deallocate(pointer p, size_type /*num*/)
  {
    if (dataMapped == nullptr) {
      Eigen::internal::aligned_free(p);
    }
  }

  template <class U, class... Args>
  void
  construct(U* p, Args&&... args)
  {
    if (dataMapped == nullptr) {
      std::allocator<T>::construct(p, std::forward<Args>(args)...);
    }
  }

  template <class U>
  void
  destroy(U* p)
  {
    if (dataMapped == nullptr) {
      std::allocator<T>::destroy(p);
    }
  }
};

} // namespace tk

#endif /* COMMON_INCLUDE_PCL_TK_ALLOCATOR */
