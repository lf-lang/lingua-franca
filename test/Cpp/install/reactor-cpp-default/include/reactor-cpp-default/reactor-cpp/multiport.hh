/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MULTIPORT_HH
#define REACTOR_CPP_MULTIPORT_HH

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iostream>
#include <type_traits>
#include <vector>

#include "assert.hh"
#include "fwd.hh"

namespace reactor {

class BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions,-warnings-as-errors
private:
  std::atomic<std::size_t> size_{0};
  std::vector<std::size_t> present_ports_{};

  // record that the port with the given index has been set
  inline void set_present(std::size_t index);

  // reset the list of set port indexes
  inline void reset() noexcept { size_.store(0, std::memory_order_relaxed); }

  [[nodiscard]] auto get_set_callback(std::size_t index) noexcept -> PortCallback;
  const PortCallback clean_callback_{[this]([[maybe_unused]] const BasePort& port) { this->reset(); }};

  [[nodiscard]] auto get_clean_callback() const noexcept -> const PortCallback& { return clean_callback_; }

protected:
  [[nodiscard]] inline auto present_ports() const -> const auto& { return present_ports_; }
  [[nodiscard]] inline auto present_ports_size() const -> auto { return size_.load(); }

  inline void present_ports_reserve(size_t n) { present_ports_.reserve(n); }

  void register_port(BasePort& port, size_t idx);

public:
  BaseMultiport() = default;
  ~BaseMultiport() = default;
};

template <class T, class A = std::allocator<T>>
class Multiport : public BaseMultiport { // NOLINT cppcoreguidelines-special-member-functions
protected:
  std::vector<T> ports_{}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

public:
  using value_type = typename A::value_type;
  using size_type = typename A::size_type;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  Multiport() noexcept = default;
  ~Multiport() noexcept = default;

  auto operator==(const Multiport& other) const noexcept -> bool {
    return std::equal(std::begin(ports_), std::end(ports_), std::begin(other.ports_), std::end(other.ports_));
  }
  auto operator!=(const Multiport& other) const noexcept -> bool { return !(*this == other); };
  inline auto operator[](std::size_t index) noexcept -> T& { return ports_[index]; }
  inline auto operator[](std::size_t index) const noexcept -> const T& { return ports_[index]; }

  inline auto begin() noexcept -> iterator { return ports_.begin(); };
  inline auto begin() const noexcept -> const_iterator { return ports_.begin(); };
  inline auto cbegin() const noexcept -> const_iterator { return ports_.cbegin(); };
  inline auto end() noexcept -> iterator { return ports_.end(); };
  inline auto end() const noexcept -> const_iterator { return ports_.end(); };
  inline auto cend() const noexcept -> const_iterator { return ports_.cend(); };

  inline auto size() const noexcept -> size_type { return ports_.size(); };
  [[nodiscard]] inline auto empty() const noexcept -> bool { return ports_.empty(); };

  [[nodiscard]] inline auto present_indices_unsorted() const noexcept -> std::vector<std::size_t> {
    return std::vector<std::size_t>(std::begin(present_ports()), std::begin(present_ports()) + present_ports_size());
  }

  [[nodiscard]] inline auto present_indices_sorted() const noexcept -> std::vector<std::size_t> {
    std::vector<std::size_t> indices(std::begin(present_ports()), std::begin(present_ports()) + present_ports_size());
    std::sort(std::begin(indices), std::end(indices));
    return indices;
  }
};

template <class T, class A = std::allocator<T>> class ModifableMultiport : public Multiport<T, A> { // NOLINT
public:
  ModifableMultiport()
      : Multiport<T>() {} // NOLINT
  ~ModifableMultiport() = default;

  inline void reserve(std::size_t size) noexcept {
    this->ports_.reserve(size);
    this->present_ports_reserve(size);
  }

  inline void push_back(const T& elem) noexcept {
    this->ports_.push_back(elem);
    this->register_port(this->ports_.back(), this->ports_.size() - 1);
  }

  template <class... Args> inline void emplace_back(Args&&... args) noexcept {
    this->ports_.emplace_back(args...);
    this->register_port(this->ports_.back(), this->ports_.size() - 1);
  }
};
} // namespace reactor

#endif // REACTOR_CPP_MULTIPORT_HH
