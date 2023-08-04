/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ASSERT_HH
#define REACTOR_CPP_ASSERT_HH

#ifdef REACTOR_CPP_VALIDATE
constexpr bool runtime_validation = true;
#else
constexpr bool runtime_validation = false;
#endif

#ifdef NDEBUG
constexpr bool runtime_assertion = false;
#else
constexpr bool runtime_assertion = true;
#endif

#include "environment.hh"

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef __linux__
#include <execinfo.h>
#include <unistd.h>
#endif

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define reactor_assert(x) assert(x)

namespace reactor {
using EnvPhase = Environment::Phase;

class ValidationError : public std::runtime_error {
private:
  static auto build_message(std::string_view msg) noexcept -> std::string;

public:
  explicit ValidationError(const std::string_view msg)
      : std::runtime_error(build_message(msg)) {}
};

#ifdef __linux__
constexpr std::size_t MAX_STACK_SIZE{10};

inline void print_debug_backtrace() {
  void* array[10]; // NOLINT
  // get void*'s for all entries on the stack
  int size = backtrace((void**)array, MAX_STACK_SIZE);
  backtrace_symbols_fd((void**)array, size, STDERR_FILENO);
}
#endif

constexpr inline void validate([[maybe_unused]] bool condition, [[maybe_unused]] const std::string_view message) {
  if constexpr (runtime_validation) { // NOLINT
    if (!condition) {
#ifdef __linux__
      print_debug_backtrace();
#endif
      throw ValidationError(message);
    }
  }
}

template <typename E> constexpr auto extract_value(E enum_value) -> typename std::underlying_type<E>::type {
  return static_cast<typename std::underlying_type<E>::type>(enum_value);
}

inline void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] EnvPhase phase) {
  if constexpr (runtime_assertion) { // NOLINT
    if (ptr->environment()->phase() != phase) {
      auto enum_value_to_name = [](EnvPhase phase) -> std::string {
        const std::map<EnvPhase, std::string> conversation_map = {
            // NOLINT
            {EnvPhase::Construction, "Construction"}, {EnvPhase::Assembly, "Assembly"},
            {EnvPhase::Startup, "Startup"},           {EnvPhase::Execution, "Execution"},
            {EnvPhase::Shutdown, "Shutdown"},         {EnvPhase::Deconstruction, "Deconstruction"}};
        // in C++20 use .contains()
        if (conversation_map.find(phase) != std::end(conversation_map)) {
          return conversation_map.at(phase);
        }
        return "Unknown Phase: Value: " + std::to_string(extract_value(phase));
      };
#ifdef __linux__
      print_debug_backtrace();
#endif

      // C++20 std::format
      throw ValidationError("Expected Phase: " + enum_value_to_name(phase) +
                            " Current Phase: " + enum_value_to_name(ptr->environment()->phase()));
    }
  }
}
} // namespace reactor

#endif // REACTOR_CPP_ASSERT_HH
