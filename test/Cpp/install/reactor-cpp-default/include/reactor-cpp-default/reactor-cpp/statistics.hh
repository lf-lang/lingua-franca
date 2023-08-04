/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_STATISTICS_HH
#define REACTOR_CPP_STATISTICS_HH

#include <atomic>

#include "reactor-cpp/config.hh"
#include "reactor-cpp/logging.hh"

namespace reactor {

class Statistics {
private:
#ifdef REACTOR_CPP_PRINT_STATISTICS
  constexpr static bool enabled_{true};
#else
  constexpr static bool enabled_{false};
#endif

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t reactor_instances_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t connections_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t reactions_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t actions_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t ports_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t processed_events_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t processed_reactions_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t triggered_actions_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t set_ports_{0};
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  inline static std::atomic_size_t scheduled_actions_{0};

  inline static void increment(std::atomic_size_t& counter) {
    if constexpr (enabled_) {
      counter.fetch_add(1, std::memory_order_release);
    }
  }

public:
  inline static void increment_reactor_instances() { increment(reactor_instances_); }
  inline static void increment_connections() { increment(connections_); }
  inline static void increment_reactions() { increment(reactions_); }
  inline static void increment_actions() { increment(actions_); }
  inline static void increment_ports() { increment(ports_); }
  inline static void increment_processed_events() { increment(processed_events_); }
  inline static void increment_processed_reactions() { increment(processed_reactions_); }
  inline static void increment_triggered_actions() { increment(triggered_actions_); }
  inline static void increment_set_ports() { increment(set_ports_); }
  inline static void increment_scheduled_actions() { increment(scheduled_actions_); }

  inline static auto reactor_instances() { return reactor_instances_.load(std::memory_order_acquire); }
  inline static auto connections() { return connections_.load(std::memory_order_acquire); }
  inline static auto reactions() { return reactions_.load(std::memory_order_acquire); }
  inline static auto actions() { return actions_.load(std::memory_order_acquire); }
  inline static auto ports() { return ports_.load(std::memory_order_acquire); }
  inline static auto processed_events() { return processed_events_.load(std::memory_order_acquire); }
  inline static auto processed_reactions() { return processed_reactions_.load(std::memory_order_acquire); }
  inline static auto triggered_actions() { return triggered_actions_.load(std::memory_order_acquire); }
  inline static auto set_ports() { return set_ports_.load(std::memory_order_acquire); }
  inline static auto scheduled_actions() { return scheduled_actions_.load(std::memory_order_acquire); }

  inline static void print() {
    if constexpr (enabled_) {
      reactor::log::Info() << "-----------------------------------------------------------";
      reactor::log::Info() << "Program statistics:";
      reactor::log::Info() << "  - number of reactors:    " << reactor_instances();
      reactor::log::Info() << "  - number of connections: " << connections();
      reactor::log::Info() << "  - number of reactions    " << reactions();
      reactor::log::Info() << "  - number of actions:     " << actions();
      reactor::log::Info() << "  - number of ports:       " << ports();
      reactor::log::Info() << "Execution statistics:";
      reactor::log::Info() << "  - processed events:      " << processed_events();
      reactor::log::Info() << "  - triggered actions:     " << triggered_actions();
      reactor::log::Info() << "  - processed reactions:   " << processed_reactions();
      reactor::log::Info() << "  - set ports:             " << set_ports();
      reactor::log::Info() << "  - scheduled actions:     " << scheduled_actions();
      reactor::log::Info() << "-----------------------------------------------------------";
    }
  }
};

} // namespace reactor

#endif // REACTOR_CPP_STATISTICS_HH
