/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_SAFE_VECTOR_HH
#define REACTOR_CPP_SAFE_VECTOR_HH

#include <atomic>
#include <cstddef>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace reactor {

template <class T> class SafeVector {
private:
  static constexpr std::size_t size_increment_{100};
  std::atomic<std::size_t> write_pos_{0};
  std::size_t vector_size_{size_increment_};
  std::vector<T> vector_{size_increment_};
  std::shared_mutex mutex_;

public:
  void push_back(const T& value) {
    auto pos = write_pos_.fetch_add(1, std::memory_order_release);

    {
      std::shared_lock<std::shared_mutex> shared_lock(mutex_);
      if (pos < vector_size_) {
        vector_[pos] = value;
      } else {
        shared_lock.unlock();
        {
          std::unique_lock<std::shared_mutex> unique_lock(mutex_);
          while (pos >= vector_size_) {
            vector_size_ += size_increment_;
            vector_.resize(vector_size_);
          }
          vector_[pos] = value;
        }
      }
    }
  }

  // careful! Using the iterators is only safe if no more elements are added to the vector!
  auto begin() -> auto { return vector_.begin(); }
  auto end() -> auto { return vector_.begin() + write_pos_.load(std::memory_order_acquire); }

  auto size() -> std::size_t { return write_pos_.load(std::memory_order_acquire); }
  auto empty() -> bool { return size() == 0; }

  void clear() { write_pos_.store(0, std::memory_order_release); }
};

} // namespace reactor

#endif // REACTOR_CPP_SAFE_VECTOR_HH
