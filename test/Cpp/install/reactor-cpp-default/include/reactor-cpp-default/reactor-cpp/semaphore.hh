/*
 * Copyright (C) 2021 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_SEMAPHORE_HH
#define REACTOR_CPP_SEMAPHORE_HH

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace reactor {

class Semaphore {
private:
  int count_;
  std::mutex mutex_{};
  std::condition_variable cv_{};

public:
  explicit Semaphore(int count)
      : count_(count) {}

  void release(int increment) {
    {
      std::lock_guard<std::mutex> lock_guard(mutex_);
      count_ += increment;
    }
    cv_.notify_all();
  }

  void acquire() {
    std::unique_lock<std::mutex> lock_guard(mutex_);
    cv_.wait(lock_guard, [&]() { return count_ != 0; });
    count_--;
  }
};

} // namespace reactor

#endif // REACTOR_CPP_SEMAPHORE_HH
