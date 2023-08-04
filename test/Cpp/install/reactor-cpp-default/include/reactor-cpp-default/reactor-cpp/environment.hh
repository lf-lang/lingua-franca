/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ENVIRONMENT_HH
#define REACTOR_CPP_ENVIRONMENT_HH

#include <set>
#include <string>
#include <vector>

#include "fwd.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/time.hh"
#include "reactor.hh"
#include "scheduler.hh"

namespace reactor {

constexpr unsigned int default_number_worker = 1;
constexpr unsigned int default_max_reaction_index = 0;
constexpr bool default_run_forever = false;
constexpr bool default_fast_fwd_execution = false;

class Environment {
public:
  enum class Phase { Construction = 0, Assembly = 1, Startup = 2, Execution = 3, Shutdown = 4, Deconstruction = 5 };

private:
  using Dependency = std::pair<Reaction*, Reaction*>;

  const std::string name_{};
  const log::NamedLogger log_;
  const unsigned int num_workers_{default_number_worker};
  unsigned int max_reaction_index_{default_max_reaction_index};
  bool run_forever_{default_run_forever};
  const bool fast_fwd_execution_{default_fast_fwd_execution};

  std::set<Reactor*> top_level_reactors_{};
  /// Set of actions that act as an input to the reactor program in this environment
  std::set<BaseAction*> input_actions_{};
  std::set<Reaction*> reactions_{};
  std::vector<Dependency> dependencies_{};

  /// The environment containing this environment. nullptr if this is the top environment
  Environment* containing_environment_{nullptr};
  /// Set of all environments contained by this environment
  std::set<Environment*> contained_environments_{};
  /// Pointer to the top level environment
  Environment* top_environment_{nullptr};

  Scheduler scheduler_;
  Phase phase_{Phase::Construction};
  Tag start_tag_{};

  const Duration timeout_{};

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

  std::mutex shutdown_mutex_{};

  auto startup(const TimePoint& start_time) -> std::thread;

public:
  explicit Environment(unsigned int num_workers, bool fast_fwd_execution = default_fast_fwd_execution,
                       const Duration& timeout = Duration::max());
  explicit Environment(const std::string& name, Environment* containing_environment);

  auto name() -> const std::string& { return name_; }

  void register_reactor(Reactor* reactor);
  void register_input_action(BaseAction* action);
  void assemble();
  auto startup() -> std::thread;
  void sync_shutdown();
  void async_shutdown();

  // Debugging methods
  void export_dependency_graph(const std::string& path);
  void dump_to_yaml(const std::string& path);

  static void dump_trigger_to_yaml(std::ofstream& yaml, const BaseAction& trigger);
  static void dump_instance_to_yaml(std::ofstream& yaml, const Reactor& reactor);
  static void dump_port_to_yaml(std::ofstream& yaml, const BasePort& port);
  static void dump_reaction_to_yaml(std::ofstream& yaml, const Reaction& reaction);

  [[nodiscard]] auto top_level_reactors() const noexcept -> const auto& { return top_level_reactors_; }
  [[nodiscard]] auto phase() const noexcept -> Phase { return phase_; }
  [[nodiscard]] auto scheduler() const noexcept -> const Scheduler* { return &scheduler_; }

  auto scheduler() noexcept -> Scheduler* { return &scheduler_; }

  [[nodiscard]] auto logical_time() const noexcept -> const LogicalTime& { return scheduler_.logical_time(); }
  [[nodiscard]] auto start_tag() const noexcept -> const Tag& { return start_tag_; }
  [[nodiscard]] auto timeout() const noexcept -> const Duration& { return timeout_; }

  static auto physical_time() noexcept -> TimePoint { return get_physical_time(); }

  [[nodiscard]] auto num_workers() const noexcept -> unsigned int { return num_workers_; }
  [[nodiscard]] auto fast_fwd_execution() const noexcept -> bool { return fast_fwd_execution_; }
  [[nodiscard]] auto run_forever() const noexcept -> bool { return run_forever_; }
  [[nodiscard]] auto max_reaction_index() const noexcept -> unsigned int { return max_reaction_index_; }

  friend Scheduler;
};
} // namespace reactor

#endif // REACTOR_CPP_ENVIRONMENT_HH
