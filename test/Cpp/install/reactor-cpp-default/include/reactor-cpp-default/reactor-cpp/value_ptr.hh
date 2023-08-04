/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

/**
 * @file Defines classes for smart value pointers as well as comparison
 * operators and factory functions.
 */

#ifndef REACTOR_CPP_VALUE_PTR_HH
#define REACTOR_CPP_VALUE_PTR_HH

#include <cstddef>
#include <memory>
#include <type_traits>

#include "reactor-cpp/logging.hh"

namespace reactor {

namespace detail {

template <class T, bool is_trivial> class ImmutableValuePtr {};
template <class T, bool is_trivial> class MutableValuePtr {};

constexpr std::size_t SIZE_THRESHOLD = 64;

template <class T> constexpr auto is_trivial() -> bool {
  return std::is_default_constructible<T>::value && std::is_trivially_copyable<T>::value && sizeof(T) <= SIZE_THRESHOLD;
}

} // namespace detail

template <class T> using MutableValuePtr = detail::MutableValuePtr<T, detail::is_trivial<T>()>;
template <class T> using ImmutableValuePtr = detail::ImmutableValuePtr<T, detail::is_trivial<T>()>;

/**
 * @rst
 * Create an instance of :class:`ImmutableValuePtr`.
 *
 * Creates and initializes a new instance of ``T`` and returns a new
 * :class:`ImmutableValuePtr` owning this value. This is analogues to
 * :std-memory:`make_shared`
 * @endrst
 * @tparam T type of the value to be created
 * @tparam Args types of ``T``'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given ``args``.
 * @param args Arguments to be forwarded to ``T``'s constructor
 * @return a new immutable value pointer
 */
template <class T, class... Args> auto make_immutable_value(Args&&... args) -> ImmutableValuePtr<T> {
  if constexpr (detail::is_trivial<T>()) {
    return ImmutableValuePtr<T>(T(std::forward<Args>(args)...));
  } else {
    return ImmutableValuePtr<T>(std::make_shared<T>(std::forward<Args>(args)...));
  }
}

/**
 * @rst
 * Create an instance of :class:`MutableValuePtr`.

 * Creates and initializes a new instance of ``T`` and returns a new
 * :class:`MutableValuePtr` owning this value. This is analogues to
 * :std-memory:`make_unique`.
 * @endrst
 * @tparam T type of the value to be created
 * @tparam Args types of ``T``'s constructor arguments. Usually, this does not
 * need to be given explicitly and will be inferred automatically from the
 * given ``args``.
 * @param args Arguments to be forwarded to ``T``'s constructor
 * @return a new mutable value pointer
 */
template <class T, class... Args> auto make_mutable_value(Args&&... args) -> MutableValuePtr<T> {
  if constexpr (detail::is_trivial<T>()) {
    return MutableValuePtr<T>(T(std::forward<Args>(args)...));
  } else {
    return MutableValuePtr<T>(std::make_unique<T>(std::forward<Args>(args)...));
  }
}

namespace detail {

/**
 * @brief Smart pointer to a mutable value.
 *
 * @rst
 * Manages the lifetime of a value in conjunction with
 * :class:`ImmutableValuePtr`. Implements ownership semantics and enforces
 * unique ownership of a mutable value. :class:`MutableValuePtr` internally
 * wraps around :std-memory:`unique_ptr`. The unique ownership ensures that no
 * other reactor can reference the value while it is allowed to change.  In
 * order to share the associated value, an instance of :class:`MutableValuePtr`
 * needs to be converted to an :class:`ImmutableValuePtr` making the associated
 * value immutable.
 * @endrst
 * @tparam T type of the value managed by this class
 * @author Christian Menard
 */
template <class T> class MutableValuePtr<T, false> {
private:
  /// The internal unique smart pointer that this class builds upon.
  std::unique_ptr<T> internal_ptr;

  /**
   * Constructor from an existing raw pointer.
   *
   * @rst
   * Constructs a :class:`MutableValuePtr` such that is obtains ownership of
   * ``value``.  This is intended only for usage by the
   * :func:`make_mutable_value()` factory function and the
   * :func:`make_immutable_copy()` method of :class:`ImmutableValuePtr`.
   * @endrst
   */
  explicit MutableValuePtr(std::unique_ptr<T>&& value)
      : internal_ptr(std::move(value)) {}

public:
  /**
   * Default constructor.
   * @rst
   * Constructs a :class:`MutableValuePtr` that owns nothing.
   * @endrst
   */
  constexpr MutableValuePtr() = default;
  ~MutableValuePtr() = default;

  /**
   * Copy constructor (Deleted).
   * @rst
   * Since :class:`MutableValuePtr` enforces unique ownership, there cannot
   * be two instances pointing to the same object and therefore copying cannot
   * be allowed,
   * @endrst
   */
  MutableValuePtr(const MutableValuePtr&) = delete;

  /**
   * Move constructor.
   * @rst
   * Constructs a :class:`MutableValuePtr` by transferring ownership from
   * another :class:`MutableValuePtr` instance ``ptr``. ``ptr`` looses
   * ownership and will own nothing.
   * @endrst
   */
  MutableValuePtr(MutableValuePtr&& ptr) noexcept = default;

  /**
   * Constructor from ``nullptr``.
   * @rst
   * Constructs a :class:`MutableValuePtr<T>` that owns nothing.
   * @endrst
   */
  explicit constexpr MutableValuePtr(std::nullptr_t)
      : internal_ptr(nullptr) {}

  /**
   * Move assignment operator.
   * @rst
   * Transfers ownership from ``ptr`` to this :class:`MutableValuePtr`
   * instance. If this instance previously owned a value, the value is deleted.
   * @endrst
   */
  auto operator=(MutableValuePtr&& ptr) noexcept -> MutableValuePtr& {
    this->internal_ptr = std::move(ptr.internal_ptr);
    return *this;
  }

  /**
   * Copy assignment operator (Deleted).
   * @rst
   * Since :class:`MutableValuePtr` enforces unique ownership, there cannot
   * be two instances pointing to the same object and therefore copying cannot
   * be allowed,
   * @endrst
   */
  auto operator=(const MutableValuePtr& ptr) -> MutableValuePtr& = delete;

  /**
   * Assignment operator from ``nullptr``.
   *
   * Releases ownership. If this instance previously owned a value, the
   * value is deleted.
   */
  auto operator=(std::nullptr_t) noexcept -> MutableValuePtr& {
    this->internal_ptr = nullptr;
    return *this;
  }

  /**
   * Retrieve a raw pointer to the managed value.
   */
  [[nodiscard]] auto get() const noexcept -> T* { return internal_ptr.get(); }

  /**
   * Cast to ``bool``. Checks if there is an associated value.
   *
   * @return ``false`` if there is no associated value (``internal_ptr ==
   *      nullptr``), ``true`` otherwise
   */
  explicit operator bool() const { return get() == nullptr; }

  /**
   * Dereference the pointer to the managed value.
   *
   * The behavior is undefined if ``get() == nullptr``.
   */
  auto operator*() const -> T& { return *get(); }
  /**
   * Dereference the pointer to the managed value.
   *
   * Provides access to members of the associated value via ``->``. The
   * behavior is undefined if ``get() == nullptr``.
   */
  auto operator->() const -> T* { return get(); } // NOLINT

  // Give ImmutableValuePtr access to the private constructor. This is required
  // for creating a MutableValuePtr from an ImmutableValuePtr in
  // get_mutable_copy()
  friend class ImmutableValuePtr<T, false>;

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto reactor::make_mutable_value(Args&&... args) -> reactor::MutableValuePtr<U>;
};

/**
 * @brief Smart pointer to an immutable value.
 *
 * @rst
 * Manages the lifetime of a value in conjunction with
 * :class:`MutableValuePtr`. Implements ownership semantics and allows shared
 * ownership of an immutable value.  :class:`ImmutableValuePtr` internally
 * wraps around :std-memory:`shared_ptr`. The shared ownership semantics
 * enables multiple reactors to share a value which is only safe if the value
 * is immutable.  In order to modify the associated value, an instance of
 * :class:`ImmutableValuePtr` needs to be converted to an
 * :class:`MutableValuePtr` making the associated value mutable. This can be
 * achieved by calling :func:`get_mutable_copy()`.
 * @endrst
 * @tparam T type of the value managed by this class
 * @author Christian Menard
 */
template <class T> class ImmutableValuePtr<T, false> {
public:
  /// A type alias that adds ``const`` to ``T``
  using const_T = typename std::add_const<T>::type;

private:
  /// The internal shared smart pointer that this class builds upon.
  std::shared_ptr<T> internal_ptr;

  /**
   * Constructor from an existing raw pointer.
   *
   * @rst
   * Constructs an :class:`ImutableValuePtr<T>` such that is obtains ownership
   * of ``value``.  This is intended only for usage by the
   * :func:`make_immutable_value()` factory function.
   * @endrst
   */
  explicit ImmutableValuePtr(std::shared_ptr<T>&& value)
      : internal_ptr(std::move(value)) {}

public:
  /**
   * Default constructor.
   * @rst
   * Constructs an :class:`ImmutableValuePtr<T>` that owns nothing.
   * @endrst
   */
  constexpr ImmutableValuePtr()
      : internal_ptr(nullptr) {}

  ~ImmutableValuePtr() = default;

  /**
   * Copy constructor.
   * @rst
   * Constructs an :class:`ImmutableValuePtr` by copying another
   * :class:`ImmutableValuePtr` instance ``ptr``. Both pointers have the same
   * associated value and, therefore, share ownership.
   * @endrst
   */
  ImmutableValuePtr(const ImmutableValuePtr& ptr) = default;
  /**
   * Move constructor.
   * @rst
   * Constructs an :class:`ImmutableValuePtr` by transferring ownership from
   * another :class:`ImmutableValuePtr` instance ``ptr``. ``ptr`` looses
   * ownership and will own nothing.
   * @endrst
   */
  ImmutableValuePtr(ImmutableValuePtr&& ptr) noexcept = default;
  /**
   * Constructor from ``nullptr``.
   * @rst
   * Constructs an :class:`ImmutableValuePtr<T>` that owns nothing.
   * @endrst
   */
  explicit constexpr ImmutableValuePtr(std::nullptr_t)
      : internal_ptr(nullptr) {}
  /**
   * @rst
   * Move constructor from :class:`MutableValuePtr`.
   *
   * Constructs an :class:`ImmutableValuePtr` by transferring ownership from a
   * :class:`MutableValuePtr` instance ``ptr``. ``ptr`` looses ownership and
   * will own nothing. This effectively converts the mutable value initially
   * associated with ``ptr`` to an immutable value.
   * @endrst
   */
  explicit ImmutableValuePtr(MutableValuePtr<T, false>&& ptr)
      : internal_ptr(std::move(ptr.internal_ptr)) {}

  /**
   * Assignment operator from ``nullptr``.
   *
   * @rst
   * Releases ownership. If this instance previously owned a value that is not
   * owned by any other instance of class:`ImmutableValuePtr`, the value is
   * deleted.
   * @endrst
   */
  auto operator=(std::nullptr_t) -> ImmutableValuePtr& {
    this->internal_ptr = nullptr;
    return *this;
  }
  /**
   * @rst
   * Assignment operator from another :class:`ImmutableValuePtr`.
   *
   * Replaces the managed value of this instance by the one managed by
   * ``ptr``. Both instances share the ownership. If this instance previously
   * owned a value that is not owned by any other instance of
   * class:`ImmutableValuePtr`, the value is deleted.
   * @endrst
   */
  auto operator=(const ImmutableValuePtr& ptr) -> ImmutableValuePtr& { // NOLINT(cert-oop54-cpp)
    this->internal_ptr = ptr.internal_ptr;
    return *this;
  }
  /**
   * @rst
   * Move assignment operator from another :class:`ImmutableValuePtr`.
   *
   * Replaces the managed value of this instance by the one managed by ``ptr``.
   * This moves the ownership from ``ptr`` to this instance. If this instance
   * previously owned a value that is not owned by any other instance of
   * class:`ImmutableValuePtr`, the value is deleted.
   * @endrst
   */
  auto operator=(ImmutableValuePtr&& ptr) noexcept -> ImmutableValuePtr& {
    this->internal_ptr = std::move(ptr.internal_ptr);
    return *this;
  }

  /**
   * Retrieve a raw pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   */
  [[nodiscard]] auto get() const -> const_T* { return internal_ptr.get(); }

  /**
   * Cast to ``bool``. Checks if there is an associated value.
   *
   * @return ``false`` if there is no associated value (``internal_ptr ==
   *      nullptr``), ``true`` otherwise
   */
  explicit operator bool() const { return get() == nullptr; }

  /**
   * Dereference the pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   *
   * The behavior is undefined if ``get() == nullptr``.
   */
  auto operator*() const -> const_T& { return *get(); }
  /**
   * Dereference the pointer to the managed value.
   *
   * Since the associated value is immutable, this only provides const access
   * to the value.
   *
   * Provides access to members of the associated value via ``->``. The
   * behavior is undefined if ``get() == nullptr``.
   */
  auto operator->() const -> const_T* { return get(); }

  /**
   * Create a mutable copy of the value associated with this instance.
   *
   * @rst
   * This is the only allowed mechanism to convert a :class:`ImmutableValuePtr`
   * to a :class:`MutableValuePtr`. In fact, it does not perform a conversion
   * but copies the associated value of this instance and gives ownership of
   * the copy to a newly created :class:`MutableValuePtr`.
   * @endrst
   *
   * Requires that ``T`` is copy constructable. The behavior is undefined if
   * ``get() == nullptr``.
   * @return a mutable value pointer
   */
  [[nodiscard]] auto get_mutable_copy() const -> MutableValuePtr<T, false> {
    return MutableValuePtr<T, false>(std::make_unique<T>(*internal_ptr));
  }

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto reactor::make_immutable_value(Args&&... args) -> reactor::ImmutableValuePtr<U>;
};

template <class T> class MutableValuePtr<T, true> {
private:
  T value_{};
  bool valid_{false};

  explicit MutableValuePtr(const T& value)
      : value_{value}
      , valid_{true} {}

public:
  constexpr MutableValuePtr() = default;
  ~MutableValuePtr() = default;
  MutableValuePtr(const MutableValuePtr&) = delete;
  MutableValuePtr(MutableValuePtr&& ptr) noexcept = default;

  explicit constexpr MutableValuePtr(std::nullptr_t) {}

  auto operator=(MutableValuePtr&& ptr) noexcept -> MutableValuePtr& = default;
  auto operator=(const MutableValuePtr& ptr) -> MutableValuePtr& = delete;

  auto operator=(std::nullptr_t) noexcept -> MutableValuePtr& {
    valid_ = false;
    return *this;
  }

  [[nodiscard]] auto get() noexcept -> T* { return valid_ ? &value_ : nullptr; }
  [[nodiscard]] auto get() const noexcept -> const T* { return valid_ ? &value_ : nullptr; }

  explicit operator bool() const { return valid_; }

  auto operator*() -> T& { return value_; }
  auto operator*() const -> const T& { return value_; }

  auto operator->() -> T* { return get(); }
  auto operator->() const -> const T* { return get(); }

  // Give ImmutableValuePtr access to the private constructor. This is required
  // for creating a MutableValuePtr from an ImmutableValuePtr in
  // get_mutable_copy()
  friend class ImmutableValuePtr<T, true>;

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto reactor::make_mutable_value(Args&&... args) -> reactor::MutableValuePtr<U>;
};

template <class T> class ImmutableValuePtr<T, true> {
public:
  /// A type alias that adds ``const`` to ``T``
  using const_T = typename std::add_const<T>::type;

private:
  T value_{};
  bool valid_{false};

  explicit ImmutableValuePtr(T value)
      : value_{value}
      , valid_{true} {}

public:
  constexpr ImmutableValuePtr() = default;
  ~ImmutableValuePtr() = default;
  ImmutableValuePtr(const ImmutableValuePtr& ptr) = default;
  ImmutableValuePtr(ImmutableValuePtr&& ptr) noexcept = default;

  explicit constexpr ImmutableValuePtr(std::nullptr_t) {}
  explicit ImmutableValuePtr(MutableValuePtr<T, true>&& ptr)
      : value_(ptr.value_)
      , valid_(ptr.valid_) {}

  auto operator=(std::nullptr_t) -> ImmutableValuePtr& {
    this->valid_ = false;
    return *this;
  }
  auto operator=(const ImmutableValuePtr& ptr) -> ImmutableValuePtr& = default;
  auto operator=(ImmutableValuePtr&& ptr) noexcept -> ImmutableValuePtr& = default;

  [[nodiscard]] auto get() const -> const_T* { return valid_ ? &value_ : nullptr; }

  explicit operator bool() const { return valid_; }

  auto operator*() const -> const_T& { return value_; }
  auto operator->() const -> const_T* { return get(); }

  [[nodiscard]] auto get_mutable_copy() const -> MutableValuePtr<T, true> { return MutableValuePtr<T, true>(*get()); }

  // Give the factory function make_mutable_value() access to the private
  // constructor
  template <class U, class... Args>
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend auto reactor::make_immutable_value(Args&&... args) -> reactor::ImmutableValuePtr<U>;
};

// Comparison operators

template <class T, class U, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() == ptr2.get();
}
template <class T, bool is_trivial>
auto operator==(const MutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) noexcept -> bool {
  return ptr1.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(std::nullptr_t, const MutableValuePtr<T, is_trivial>& ptr2) noexcept -> bool {
  return ptr2.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(const ImmutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) noexcept -> bool {
  return ptr1.get() == nullptr;
}
template <class T, bool is_trivial>
auto operator==(std::nullptr_t, const ImmutableValuePtr<T, is_trivial>& ptr1) noexcept -> bool {
  return ptr1.get() == nullptr;
}

template <class T, class U, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) noexcept
    -> bool {
  return ptr1.get() != ptr2.get();
}

template <class T, class U, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, const MutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, class U, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, const ImmutableValuePtr<U, is_trivial>& ptr2) -> bool {
  return ptr1.get() != ptr2.get();
}
template <class T, bool is_trivial>
auto operator!=(const MutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(std::nullptr_t, const MutableValuePtr<T, is_trivial>& ptr1) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(const ImmutableValuePtr<T, is_trivial>& ptr1, std::nullptr_t) -> bool {
  return ptr1.get() != nullptr;
}
template <class T, bool is_trivial>
auto operator!=(std::nullptr_t, const ImmutableValuePtr<T, is_trivial>& ptr1) -> bool {
  return ptr1.get() != nullptr;
}

} // namespace detail

} // namespace reactor

#endif
