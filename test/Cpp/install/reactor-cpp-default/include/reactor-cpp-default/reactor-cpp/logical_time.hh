/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_LOGICAL_TIME_HH
#define REACTOR_CPP_LOGICAL_TIME_HH

#include "time.hh"

namespace reactor {
using mstep_t = unsigned long; // at least 32 bit
class LogicalTime;

class Tag {
private:
  TimePoint time_point_{};
  mstep_t micro_step_{0};

  Tag(const TimePoint& time_point, const mstep_t& micro_step)
      : time_point_{time_point}
      , micro_step_{micro_step} {}

public:
  Tag() = default;
  Tag(Tag&&) = default;
  Tag(const Tag&) = default;
  auto operator=(const Tag&) -> Tag& = default;
  auto operator=(Tag&&) -> Tag& = default;
  ~Tag() = default;

  [[nodiscard]] auto time_point() const noexcept -> const TimePoint& { return time_point_; }
  [[nodiscard]] auto micro_step() const noexcept -> const mstep_t& { return micro_step_; }

  [[nodiscard]] static auto from_physical_time(TimePoint time_point) noexcept -> Tag;
  [[nodiscard]] static auto from_logical_time(const LogicalTime& logical_time) noexcept -> Tag;

  [[nodiscard]] static auto max_for_timepoint(TimePoint time_point) noexcept -> Tag;

  [[nodiscard]] auto delay(Duration offset = Duration::zero()) const noexcept -> Tag;
  [[nodiscard]] auto subtract(Duration offset = Duration::zero()) const noexcept -> Tag;
  [[nodiscard]] auto decrement() const noexcept -> Tag;
};

// define all the comparison operators
auto operator==(const Tag& lhs, const Tag& rhs) noexcept -> bool;
auto inline operator!=(const Tag& lhs, const Tag& rhs) noexcept -> bool { return !(lhs == rhs); }
auto operator<(const Tag& lhs, const Tag& rhs) noexcept -> bool;
auto inline operator>(const Tag& lhs, const Tag& rhs) noexcept -> bool { return rhs < lhs; }
auto inline operator<=(const Tag& lhs, const Tag& rhs) noexcept -> bool { return !(lhs > rhs); }
auto inline operator>=(const Tag& lhs, const Tag& rhs) noexcept -> bool { return !(lhs < rhs); }

class LogicalTime {
private:
  TimePoint time_point_{};
  mstep_t micro_step_{0};

public:
  void advance_to(const Tag& tag);
  void advance_to(const LogicalTime& time);

  [[nodiscard]] auto time_point() const noexcept -> TimePoint { return time_point_; }
  [[nodiscard]] auto micro_step() const noexcept -> mstep_t { return micro_step_; }
};

// c++20 starship operator will save us this mess
auto operator==(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool;
auto inline operator!=(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return !(logical_time == tag);
}
auto operator<(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool;
auto operator>(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool;
auto inline operator<=(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return !(logical_time > tag);
}
auto inline operator>=(const LogicalTime& logical_time, const Tag& tag) noexcept -> bool {
  return !(logical_time < tag);
}
auto inline operator==(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool { return logical_time == tag; }
auto inline operator!=(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool {
  return !(logical_time == tag);
}
auto inline operator<(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool { return logical_time > tag; }
auto inline operator>(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool { return logical_time < tag; }
auto inline operator<=(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool {
  return !(tag > logical_time);
}
auto inline operator>=(const Tag& tag, const LogicalTime& logical_time) noexcept -> bool {
  return !(tag < logical_time);
}

auto operator<<(std::ostream& os, const Tag& tag) -> std::ostream&;
auto operator<<(std::ostream& os, const LogicalTime& tag) -> std::ostream&;

} // namespace reactor

#endif // REACTOR_CPP_LOGICAL_TIME_HH
