/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 *   Christian Menard
 */

#ifndef REACTOR_CPP_CONNECTION_HH
#define REACTOR_CPP_CONNECTION_HH

#include "action.hh"
#include "assert.hh"
#include "environment.hh"
#include "fwd.hh"
#include "logical_time.hh"
#include "port.hh"
#include "reaction.hh"
#include "reactor.hh"
#include "time.hh"
#include "time_barrier.hh"

namespace reactor {

template <class T> class Connection : public Action<T> {
private:
  Port<T>* upstream_port_{nullptr};
  std::set<Port<T>*> downstream_ports_{};

protected:
  Connection(const std::string& name, Reactor* container, bool is_logical, Duration min_delay)
      : Action<T>(name, container, is_logical, min_delay) {}
  Connection(const std::string& name, Environment* environment, bool is_logical, Duration min_delay)
      : Action<T>(name, environment, is_logical, min_delay) {}

  [[nodiscard]] auto downstream_ports() -> auto& { return downstream_ports_; }
  [[nodiscard]] auto downstream_ports() const -> const auto& { return downstream_ports_; }
  [[nodiscard]] auto upstream_port() -> auto* { return upstream_port_; }
  [[nodiscard]] auto upstream_port() const -> const auto* { return upstream_port_; }

  virtual auto upstream_set_callback() noexcept -> PortCallback = 0;

public:
  virtual void bind_upstream_port(Port<T>* port) {
    reactor_assert(upstream_port_ == nullptr);
    upstream_port_ = port;
    port->register_set_callback(upstream_set_callback());
  }

  virtual void bind_downstream_port(Port<T>* port) {
    [[maybe_unused]] bool result = this->downstream_ports_.insert(port).second;
    reactor_assert(result);
  };
};

template <class T> class BaseDelayedConnection : public Connection<T> {
protected:
  BaseDelayedConnection(const std::string& name, Reactor* container, bool is_logical, Duration delay)
      : Connection<T>(name, container, is_logical, delay) {}
  BaseDelayedConnection(const std::string& name, Environment* environment, bool is_logical, Duration delay)
      : Connection<T>(name, environment, is_logical, delay) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      if constexpr (std::is_same<T, void>::value) {
        this->schedule();
      } else {
        this->schedule(std::move(typed_port.get()));
      }
    };
  }

public:
  void setup() noexcept override {
    Action<T>::setup();

    if constexpr (std::is_same<T, void>::value) {
      for (auto port : this->downstream_ports()) {
        port->set();
      }
    } else {
      for (auto port : this->downstream_ports()) {
        port->set(std::move(this->get()));
      }
    }
  }
};

template <class T> class DelayedConnection : public BaseDelayedConnection<T> {
public:
  DelayedConnection(const std::string& name, Reactor* container, Duration delay)
      : BaseDelayedConnection<T>(name, container, true, delay) {}
};

template <class T> class PhysicalConnection : public BaseDelayedConnection<T> {
public:
  PhysicalConnection(const std::string& name, Reactor* container, Duration delay)
      : BaseDelayedConnection<T>(name, container, false, delay) {}
};

template <class T> class EnclaveConnection : public BaseDelayedConnection<T> {
private:
  LogicalTimeBarrier logical_time_barrier_;

protected:
  log::NamedLogger log_; // NOLINT

  EnclaveConnection(const std::string& name, Environment* enclave, const Duration& delay)
      : BaseDelayedConnection<T>(name, enclave, false, delay)
      , log_{this->fqn()} {}

public:
  EnclaveConnection(const std::string& name, Environment* enclave)
      : BaseDelayedConnection<T>(name, enclave, false, Duration::zero())
      , log_{this->fqn()} {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      const auto* scheduler = port.environment()->scheduler();
      // This callback will be called from a reaction executing in the context
      // of the upstream port. Hence, we can retrieve the current tag directly
      // without locking.
      auto tag = Tag::from_logical_time(scheduler->logical_time());
      bool result{false};
      if constexpr (std::is_same<T, void>::value) {
        result = this->schedule_at(tag);
      } else {
        result = this->schedule_at(std::move(typed_port.get()), tag);
      }
      reactor_assert(result);
    };
  }

  inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, std::condition_variable& cv,
                          const std::function<bool(void)>& abort_waiting) -> bool override {
    log_.debug() << "downstream tries to acquire tag " << tag;

    if (this->upstream_port() == nullptr) {
      return true;
    }

    if (logical_time_barrier_.try_acquire_tag(tag)) {
      return true;
    }

    // Insert an empty event into the upstream event queue. This ensures that we
    // will get notified and woken up as soon as the tag becomes safe to process.
    // It is important to unlock the mutex here. Otherwise we could enter a deadlock as
    // scheduling the upstream event also requires holding the upstream mutex.
    lock.unlock();
    bool result = this->upstream_port()->environment()->scheduler()->schedule_empty_async_at(tag);
    lock.lock();

    // If inserting the empty event was not successful, then this is because the upstream
    // scheduler already processes a later event. In this case, it is safe to assume that
    // the tag is acquired.
    if (!result) {
      return true;
    }

    // Wait until we receive a release_tag message from upstream
    return logical_time_barrier_.acquire_tag(tag, lock, cv, abort_waiting);
  }

  void bind_upstream_port(Port<T>* port) override {
    Connection<T>::bind_upstream_port(port);
    port->environment()->scheduler()->register_release_tag_callback([this](const LogicalTime& tag) {
      logical_time_barrier_.release_tag(tag);
      log_.debug() << "upstream released tag " << tag;
      this->environment()->scheduler()->notify();
    });
  }
};

template <class T> class DelayedEnclaveConnection : public EnclaveConnection<T> {
public:
  DelayedEnclaveConnection(const std::string& name, Environment* enclave, Duration delay)
      : EnclaveConnection<T>(name, enclave, delay) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      const auto* scheduler = port.environment()->scheduler();
      // This callback will be called from a reaction executing in the context
      // of the upstream port. Hence, we can retrieve the current tag directly
      // without locking.
      auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(this->min_delay());
      bool result{false};
      if constexpr (std::is_same<T, void>::value) {
        result = this->schedule_at(tag);
      } else {
        result = this->schedule_at(std::move(typed_port.get()), tag);
      }
      reactor_assert(result);
    };
  }

  inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, std::condition_variable& cv,
                          const std::function<bool(void)>& abort_waiting) -> bool override {
    // Since this is a delayed connection, we can go back in time and need to
    // acquire the latest upstream tag that can create an event at the given
    // tag. We also need to consider that given a delay d and a tag g=(t, n),
    // for any value of n, g + d = (t, 0). Hence, we need to quire a tag with
    // the highest possible microstep value.
    auto upstream_tag = tag.subtract(this->min_delay());
    return EnclaveConnection<T>::acquire_tag(upstream_tag, lock, cv, abort_waiting);
  }
};

template <class T> class PhysicalEnclaveConnection : public EnclaveConnection<T> {
public:
  PhysicalEnclaveConnection(const std::string& name, Environment* enclave)
      : EnclaveConnection<T>(name, enclave) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      if constexpr (std::is_same<T, void>::value) {
        this->schedule();
      } else {
        this->schedule(std::move(typed_port.get()));
      }
    };
  }

  inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, std::condition_variable& cv,
                          const std::function<bool(void)>& abort_waiting) -> bool override {
    this->log_.debug() << "downstream tries to acquire tag " << tag;
    return PhysicalTimeBarrier::acquire_tag(tag, lock, cv, abort_waiting);
  }

  void bind_upstream_port(Port<T>* port) override { Connection<T>::bind_upstream_port(port); }
};

} // namespace reactor

#endif // REACTOR_CPP_CONNECTION_HH
