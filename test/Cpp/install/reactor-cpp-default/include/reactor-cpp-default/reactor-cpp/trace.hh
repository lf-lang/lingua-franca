/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

// NOTE: This file is named trace.hpp (with the hpp file extension) on purpose.
// This is to exclude the file from all clang-tidy checks. This is more a hacky
// workaround than an actual solution, but apparently clang-tidy does not allow
// something nicer at the moment.

#include <string>

#include "reactor-cpp/config.hh"
#include "reactor-cpp/logical_time.hh"

// Only enable tracing if REACTOR_CPP_TRACE is set.
// Also, disable tracing if clang analytics are run as it produces many errors.
#if defined(REACTOR_CPP_TRACE) && !defined(__clang_analyzer__)

#undef LTTNG_UST_COMPAT_API_VERSION
#define LTTNG_UST_COMPAT_API_VERSION 0

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER reactor_cpp

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "reactor-cpp/trace.hh"

// LTTng requires this header to be included multiple times.
#ifndef REACTOR_CPP_TRACE_HH
namespace reactor {
constexpr bool tracing_enabled = true;
}
#endif

#if !defined(REACTOR_CPP_TRACE_HH) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define REACTOR_CPP_TRACE_HH

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    reactor_cpp, reaction_execution_starts,
    TP_ARGS(int, worker_id_arg, const std::string&, reaction_name_arg, const reactor::LogicalTime&, tag_arg),
    TP_FIELDS(ctf_string(reaction_name, reaction_name_arg.c_str()) ctf_integer(int, worker_id, worker_id_arg)
                  ctf_integer(unsigned long long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                      ctf_integer(unsigned long, timestamp_microstep, tag_arg.micro_step())))

TRACEPOINT_EVENT(
    reactor_cpp, reaction_execution_finishes,
    TP_ARGS(int, worker_id_arg, const std::string&, reaction_name_arg, const reactor::LogicalTime&, tag_arg),
    TP_FIELDS(ctf_string(reaction_name, reaction_name_arg.c_str()) ctf_integer(int, worker_id, worker_id_arg)
                  ctf_integer(unsigned long long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                      ctf_integer(unsigned long, timestamp_microstep, tag_arg.micro_step())))

TRACEPOINT_EVENT(
    reactor_cpp, schedule_action,
    TP_ARGS(const std::string&, reactor_name_arg, const std::string&, action_name_arg, const reactor::Tag&, tag_arg),
    TP_FIELDS(ctf_string(reactor_name, reactor_name_arg.c_str()) ctf_string(action_name, action_name_arg.c_str())
                  ctf_integer(unsigned long long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                      ctf_integer(unsigned long, timestamp_microstep, tag_arg.micro_step())))

TRACEPOINT_EVENT(
    reactor_cpp, trigger_reaction,
    TP_ARGS(const std::string&, reactor_name_arg, const std::string&, reaction_name_arg, const reactor::LogicalTime&,
            tag_arg),
    TP_FIELDS(ctf_string(reactor_name, reactor_name_arg.c_str()) ctf_string(reaction_name, reaction_name_arg.c_str())
                  ctf_integer(unsigned long long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                      ctf_integer(unsigned long, timestamp_microstep, tag_arg.micro_step())))

#endif /* REACTOR_CPP_TRACE_HH */

#include <lttng/tracepoint-event.h>

#else

#ifndef REACTOR_CPP_TRACE_HH
// NOLINTNEXTLINE
#define REACTOR_CPP_TRACE_HH

namespace reactor {
constexpr bool tracing_enabled = false;
} // namespace reactor

// empty definition in case we compile without tracing
#define tracepoint(...)

#endif // REACTOR_CPP_TRACE_HH

#endif // REACTOR_CPP_TRACE
