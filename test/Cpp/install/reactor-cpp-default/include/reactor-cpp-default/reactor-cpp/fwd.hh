/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_FWD_HH
#define REACTOR_CPP_FWD_HH

#include <functional>

namespace reactor {

class BaseAction;
class BasePort;
class Environment;
class Reaction;
class Reactor;
class Scheduler;
class Tag;

template <class T> class Action;
template <class T> class Port;

using PortCallback = std::function<void(const BasePort&)>;

} // namespace reactor

#endif // REACTOR_CPP_FWD_HH
