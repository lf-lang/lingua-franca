/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_PORT_HH
#define REACTOR_CPP_PORT_HH

#include <set>
#include <vector>

#include "assert.hh"
#include "fwd.hh"
#include "multiport.hh"
#include "reactor.hh"
#include "value_ptr.hh"

namespace reactor {

enum class PortType { Input, Output, Delay };

class BasePort : public ReactorElement {
private:
  BasePort* inward_binding_{nullptr};
  std::set<BasePort*> outward_bindings_{};
  const PortType type_;

  std::set<Reaction*> dependencies_{};
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> anti_dependencies_{};

  PortCallback set_callback_{nullptr};
  PortCallback clean_callback_{nullptr};

protected:
  bool present_{false}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name, match_port_enum(type), container)
      , type_(type) {}

  void base_bind_to(BasePort* port);
  void register_dependency(Reaction* reaction, bool is_trigger) noexcept;
  void register_antidependency(Reaction* reaction) noexcept;
  virtual void cleanup() = 0;

  static auto match_port_enum(PortType type) noexcept -> ReactorElement::Type {
    switch (type) {
    case PortType::Output:
      return ReactorElement::Type::Output;
    case PortType::Input:
      return ReactorElement::Type::Input;
    default:
      return ReactorElement::Type::Port;
    };
  }

  inline void invoke_set_callback() noexcept {
    if (set_callback_ != nullptr) {
      set_callback_(*this);
    }
  }

  inline void invoke_clean_callback() noexcept {
    if (clean_callback_ != nullptr) {
      clean_callback_(*this);
    }
  }

public:
  [[nodiscard]] inline auto is_input() const noexcept -> bool { return type_ == PortType::Input; }
  [[nodiscard]] inline auto is_output() const noexcept -> bool { return type_ == PortType::Output; }
  [[nodiscard]] inline auto is_present() const noexcept -> bool {
    if (has_inward_binding()) {
      return inward_binding()->is_present();
    }
    return present_;
  };

  [[nodiscard]] inline auto has_inward_binding() const noexcept -> bool { return inward_binding_ != nullptr; }
  [[nodiscard]] inline auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }
  [[nodiscard]] inline auto has_dependencies() const noexcept -> bool { return !dependencies_.empty(); }
  [[nodiscard]] inline auto has_anti_dependencies() const noexcept -> bool { return !anti_dependencies_.empty(); }

  [[nodiscard]] inline auto inward_binding() const noexcept -> BasePort* { return inward_binding_; }
  [[nodiscard]] inline auto outward_bindings() const noexcept -> const auto& { return outward_bindings_; }

  [[nodiscard]] inline auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] inline auto dependencies() const noexcept -> const auto& { return dependencies_; }
  [[nodiscard]] inline auto anti_dependencies() const noexcept -> const auto& { return anti_dependencies_; }
  [[nodiscard]] inline auto port_type() const noexcept -> PortType { return type_; }

  void register_set_callback(const PortCallback& callback);
  void register_clean_callback(const PortCallback& callback);

  friend class Reaction;
  friend class Scheduler;
};

template <class T> class Port : public BasePort {
private:
  ImmutableValuePtr<T> value_ptr_{nullptr};

  void cleanup() noexcept final {
    value_ptr_ = nullptr;
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = T;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<T>* port) { base_bind_to(port); }
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<T>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<T>*>&;

  virtual void set(const ImmutableValuePtr<T>& value_ptr);
  void set(MutableValuePtr<T>&& value_ptr) { set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr))); }
  void set(const T& value) { set(make_immutable_value<T>(value)); }
  void set(T&& value) { set(make_immutable_value<T>(std::forward<T>(value))); }
  // Setting a port to nullptr is not permitted.
  void set(std::nullptr_t) = delete;
  void startup() final {}
  void shutdown() final {}

  auto get() const noexcept -> const ImmutableValuePtr<T>&;
};

template <> class Port<void> : public BasePort {
private:
  void cleanup() noexcept final {
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = void;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<void>* port) { base_bind_to(port); }
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<void>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<void>*>&;

  void set();

  void startup() final {}
  void shutdown() final {}
};

template <class T> class Input : public Port<T> { // NOLINT
public:
  Input(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Input, container) {}

  Input(Input&&) = default; // NOLINT(performance-noexcept-move-constructor)
};

template <class T> class Output : public Port<T> { // NOLINT
public:
  Output(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Output, container) {}

  Output(Output&&) = default; // NOLINT(performance-noexcept-move-constructor)
};

} // namespace reactor

#include "impl/port_impl.hh"

#endif // REACTOR_CPP_PORT_HH
