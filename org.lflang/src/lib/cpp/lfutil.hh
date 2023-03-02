/*
 * Copyright (c) 2020, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <reactor-cpp/reactor-cpp.hh>
#include <reactor-cpp/logging.hh>

namespace lfutil {

template<class T>
void after_delay(reactor::Action<T>* action, const reactor::Port<T>* port) {
  if constexpr(std::is_void<T>::value) {
    action->schedule();
  } else {
    action->schedule(std::move(port->get()));
  }
}

template<class T>
void after_forward(const reactor::Action<T>* action, reactor::Port<T>* port) {
  if constexpr(std::is_void<T>::value) {
    port->set();
  } else {
    port->set(std::move(action->get()));
  }
}

class LFScope {
 private:
  reactor::Reactor* reactor;
 public:
  LFScope(reactor::Reactor* reactor) : reactor(reactor) {}

  reactor::TimePoint get_physical_time() const { return reactor->get_physical_time(); }
  reactor::Tag get_tag() const { return reactor->get_tag(); }
  reactor::TimePoint get_logical_time() const { return reactor->get_logical_time(); }
  reactor::mstep_t get_microstep() const { return reactor->get_microstep(); }
  reactor::Duration get_elapsed_logical_time() const { return reactor->get_elapsed_logical_time(); }
  reactor::Duration get_elapsed_physical_time() const { return reactor->get_elapsed_physical_time(); }
  reactor::Environment* environment() const { return reactor->environment(); }
  void request_stop() const { return environment()->sync_shutdown(); }
};

template<class T>
void bind_multiple_ports(
    std::vector<reactor::Port<T>*>& left_ports,
    std::vector<reactor::Port<T>*>& right_ports,
    bool repeat_left) {

  if (repeat_left) {
    auto l_size = left_ports.size();
    auto r_size = right_ports.size();
    // divide and round up
    auto repetitions = r_size / l_size + (r_size % l_size != 0);
    // repeat repetitions-1 times
    left_ports.reserve(repetitions * l_size);
    for (std::size_t i{1}; i < repetitions; i++) {
      std::copy_n(left_ports.begin(), l_size, std::back_inserter(left_ports));
    }
  }

  auto left_it = left_ports.begin();
  auto right_it = right_ports.begin();

  if (left_ports.size() < right_ports.size()) {
    reactor::log::Warn() << "There are more right ports than left ports. "
                         << "Not all ports will be connected!";
  } else if (left_ports.size() > right_ports.size()) {
    reactor::log::Warn() << "There are more left ports than right ports. "
                         << "Not all ports will be connected!";
  }

  while (left_it != left_ports.end() && right_it != right_ports.end()) {
    auto left = *left_it;
    auto right = *right_it;
    left->bind_to(right);
    left_it++;
    right_it++;
  }
}

template<class T>
void bind_multiple_connections_with_ports(
    std::vector<reactor::Connection<T>*>& connections,
    std::vector<reactor::Port<T>*>& ports,
    bool repeat_left) {

  if (repeat_left) {
    auto l_size = connections.size();
    auto r_size = ports.size();
    // divide and round up
    auto repetitions = r_size / l_size + (r_size % l_size != 0);
    // repeat repetitions-1 times
    connections.reserve(repetitions * l_size);
    for (std::size_t i{1}; i < repetitions; i++) {
      std::copy_n(connections.begin(), l_size, std::back_inserter(connections));
    }
  }

  auto left_it = connections.begin();
  auto right_it = ports.begin();

  if (connections.size() < ports.size()) {
    reactor::log::Warn() << "There are more right ports than left ports. "
                         << "Not all ports will be connected!";
  } else if (connections.size() > ports.size()) {
    reactor::log::Warn() << "There are more left ports than right ports. "
                         << "Not all ports will be connected!";
  }

  while (left_it != connections.end() && right_it != ports.end()) {
    auto left = *left_it;
    auto right = *right_it;
    left->bind_downstream_port(right);
    left_it++;
    right_it++;
  }
}

}
