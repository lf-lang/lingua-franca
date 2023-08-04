/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_IMPL_PORT_IMPL_HH
#define REACTOR_CPP_IMPL_PORT_IMPL_HH

#include "../assert.hh"
#include "../environment.hh"
#include "reactor-cpp/port.hh"

namespace reactor {

template <class T> [[maybe_unused]] auto Port<T>::typed_outward_bindings() const noexcept -> const std::set<Port<T>*>& {
  // HACK this cast is ugly but should be safe as long as we only allow to
  // bind with Port<T>*. The alternative would be to copy the entire set and
  // cast each element individually, which is also ugly...
  return reinterpret_cast<const std::set<Port<T>*>&>(outward_bindings()); // NOLINT C++20 std::bit_cast
}

template <class T> auto Port<T>::typed_inward_binding() const noexcept -> Port<T>* {
  // we can use a static cast here since we know that this port is always
  // connected with another Port<T>.
  return static_cast<Port<T>*>(inward_binding());
}

template <class T> void Port<T>::set(const ImmutableValuePtr<T>& value_ptr) {
  reactor::validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                           "binding!");
  reactor::validate(value_ptr != nullptr, "Ports may not be set to nullptr!");

  auto scheduler = environment()->scheduler();
  this->value_ptr_ = std::move(value_ptr);
  scheduler->set_port(this);
  this->present_ = true;
}

template <class T> auto Port<T>::get() const noexcept -> const ImmutableValuePtr<T>& {
  if (has_inward_binding()) {
    return typed_inward_binding()->get();
  }
  return value_ptr_;
}

} // namespace reactor

#endif // REACTOR_CPP_IMPL_PORT_IMPL_HH
