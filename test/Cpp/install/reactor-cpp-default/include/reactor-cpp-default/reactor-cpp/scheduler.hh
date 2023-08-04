/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_SCHEDULER_HH
#define REACTOR_CPP_SCHEDULER_HH

#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <thread>
#include <vector>

#include "fwd.hh"
#include "logical_time.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/time.hh"
#include "safe_vector.hh"
#include "semaphore.hh"

namespace reactor {

using ReleaseTagCallback = std::function<void(const LogicalTime&)>;

// forward declarations
class Scheduler;
class Worker;

class Worker { // NOLINT
private:
  Scheduler& scheduler_;
  const unsigned int identity_{0};
  std::thread thread_{};
  log::NamedLogger log_;

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static thread_local const Worker* current_worker;

  void work() const;
  void execute_reaction(Reaction* reaction) const;

public:
  Worker(Scheduler& scheduler, unsigned int identity, const std::string& name)
      : scheduler_{scheduler}
      , identity_{identity}
      , log_(name) {}
  Worker(Worker&& worker); // NOLINT(performance-noexcept-move-constructor)
  Worker(const Worker& worker) = delete;

  void start_thread() { thread_ = std::thread(&Worker::work, this); }
  void join_thread() { thread_.join(); }

  static auto current_worker_id() -> unsigned { return current_worker->identity_; }
};

class ReadyQueue {
private:
  std::vector<Reaction*> queue_{};
  std::atomic<std::ptrdiff_t> size_{0};
  Semaphore sem_{0};
  std::ptrdiff_t waiting_workers_{0};
  const std::ptrdiff_t num_workers_;
  log::NamedLogger& log_;

public:
  explicit ReadyQueue(log::NamedLogger& log, std::ptrdiff_t num_workers)
      : num_workers_(num_workers)
      , log_(log) {}

  /**
   * Retrieve a ready reaction from the queue.
   *
   * This method may be called concurrently. In case the queue is empty, the
   * method blocks and waits until a ready reaction becomes available.
   */
  auto pop() -> Reaction*;

  /**
   * Fill the queue up with ready reactions.
   *
   * This method assumes that the internal queue is empty. It moves all
   * reactions from the provided `ready_reactions` vector to the internal
   * queue, leaving `ready_reactions` empty.
   *
   * Note that this method is not thread-safe. The caller needs to ensure that
   * no other thread will try to read from the queue during this operation.
   */
  void fill_up(std::vector<Reaction*>& ready_reactions);
};

using ActionList = SafeVector<BaseAction*>;
using ActionListPtr = std::unique_ptr<ActionList>;

class EventQueue {
private:
  std::shared_mutex mutex_;
  std::map<Tag, ActionListPtr> event_queue_;
  /// stores the actions triggered at the current tag
  ActionListPtr triggered_actions_{nullptr};

  std::vector<ActionListPtr> action_list_pool_;
  static constexpr std::size_t action_list_pool_increment_{10};

  void fill_action_list_pool();

public:
  EventQueue() { fill_action_list_pool(); }

  [[nodiscard]] auto empty() const -> bool { return event_queue_.empty(); }
  [[nodiscard]] auto next_tag() const -> Tag;

  auto insert_event_at(const Tag& tag) -> const ActionListPtr&;

  // should only be called while holding the scheduler mutex
  auto extract_next_event() -> ActionListPtr;

  // should only be called while holding the scheduler mutex
  void return_action_list(ActionListPtr&& action_list);

  // should only be called while holding the scheduler mutex
  void discard_events_until_tag(const Tag& tag);
};

class Scheduler { // NOLINT
private:
  const bool using_workers_;
  LogicalTime logical_time_{};

  Environment* environment_;
  std::vector<Worker> workers_{};
  log::NamedLogger log_;

  std::mutex scheduling_mutex_;
  std::condition_variable cv_schedule_;

  EventQueue event_queue_;
  /// stores the actions triggered at the current tag
  ActionListPtr triggered_actions_{nullptr};

  std::vector<std::vector<BasePort*>> set_ports_;
  std::vector<std::vector<Reaction*>> triggered_reactions_;
  std::vector<std::vector<Reaction*>> reaction_queue_;
  unsigned int reaction_queue_pos_{std::numeric_limits<unsigned>::max()};

  ReadyQueue ready_queue_;
  std::atomic<std::ptrdiff_t> reactions_to_process_{0};

  std::atomic<bool> stop_{false};
  bool continue_execution_{true};

  std::vector<ReleaseTagCallback> release_tag_callbacks_{};
  void release_current_tag();

  void schedule() noexcept;
  auto schedule_ready_reactions() -> bool;
  void next();
  void terminate_all_workers();
  void set_port_helper(BasePort* port);

  void advance_logical_time_to(const Tag& tag);

public:
  explicit Scheduler(Environment* env);
  ~Scheduler();

  void schedule_sync(BaseAction* action, const Tag& tag);
  auto schedule_async(BaseAction* action, const Duration& delay) -> Tag;
  auto schedule_async_at(BaseAction* action, const Tag& tag) -> bool;
  auto schedule_empty_async_at(const Tag& tag) -> bool;

  void inline notify() noexcept { cv_schedule_.notify_one(); }

  auto inline lock() noexcept -> auto { return std::unique_lock<std::mutex>(scheduling_mutex_); }

  void set_port(BasePort* port);

  [[nodiscard]] inline auto logical_time() const noexcept -> const auto& { return logical_time_; }

  void start();
  void stop();

  void register_release_tag_callback(const ReleaseTagCallback& callback);

  friend Worker;
};

} // namespace reactor

#endif // REACTOR_CPP_SCHEDULER_HH
