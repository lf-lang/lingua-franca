/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_REACTION_HH
#define REACTOR_CPP_REACTION_HH

#include <functional>
#include <set>

#include "reactor.hh"

namespace reactor {

class Reaction : public ReactorElement { // NOLINT
private:
  std::set<BaseAction*> action_triggers_;
  std::set<BaseAction*> scheduable_actions_;
  std::set<BasePort*> port_trigger_;
  std::set<BasePort*> antidependencies_;
  std::set<BasePort*> dependencies_;

  const int priority_;
  unsigned int index_{};

  std::function<void(void)> body_{nullptr};

  Duration deadline_{Duration::zero()};
  std::function<void(void)> deadline_handler_{nullptr};

  void set_deadline_impl(Duration deadline, const std::function<void(void)>& handler);

public:
  Reaction(const std::string& name, int priority, Reactor* container, std::function<void(void)> body);

  ~Reaction() override = default;

  void declare_trigger(BaseAction* action);
  void declare_trigger(BasePort* port);
  void declare_schedulable_action(BaseAction* action);
  void declare_antidependency(BasePort* port);
  void declare_dependency(BasePort* port);

  [[nodiscard]] auto action_triggers() const noexcept -> const auto& { return action_triggers_; }

  [[nodiscard]] auto port_triggers() const noexcept -> const auto& { return port_trigger_; }

  [[maybe_unused]] [[nodiscard]] auto antidependencies() const noexcept -> const auto& { return antidependencies_; }

  [[nodiscard]] auto dependencies() const noexcept -> const auto& { return dependencies_; }

  [[maybe_unused]] [[nodiscard]] auto scheduable_actions() const noexcept -> const auto& { return scheduable_actions_; }

  [[nodiscard]] auto priority() const noexcept -> int { return priority_; }

  void startup() final {}
  void shutdown() final {}
  void trigger();
  void set_index(unsigned index);

  template <class Dur> void set_deadline(Dur deadline, const std::function<void(void)>& handler) {
    set_deadline_impl(std::chrono::duration_cast<Duration>(deadline), handler);
  }

  [[nodiscard]] auto has_deadline() const noexcept -> bool { return deadline_ != Duration::zero(); }

  [[nodiscard]] auto index() const noexcept -> unsigned int { return index_; }
};

} // namespace reactor

#endif // REACTOR_CPP_REACTION_HH
