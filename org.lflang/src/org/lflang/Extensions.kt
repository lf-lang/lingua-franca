/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
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

package org.lflang

import org.lflang.lf.*
import java.lang.AssertionError


fun ReactorDecl.toDefinition(): Reactor = when (this) {
    is Reactor         -> this
    is ImportedReactor -> this.reactorClass
    else               -> throw AssertionError("unreachable")
}

/**
 * Given a reactor class, return a list of all its actions,
 * which includes actions of base classes that it extends.
 */
val Reactor.allActions: List<Action> get() = superClassRecursor { actions }

/**
 * Given a reactor class, return a list of all its connections,
 * which includes connections of base classes that it extends.
 */
val Reactor.allConnections: List<Connection> get() = superClassRecursor { connections }

/**
 * Given a reactor class, return a list of all its inputs,
 * which includes inputs of base classes that it extends.
 */
val Reactor.allInputs: List<Input> get() = superClassRecursor { inputs }

/**
 * Given a reactor class, return a list of all its outputs,
 * which includes outputs of base classes that it extends.
 */
val Reactor.allOutputs: List<Output> get() = superClassRecursor { outputs }

/**
 * Given a reactor class, return a list of all its instantiations,
 * which includes instantiations of base classes that it extends.
 */
val Reactor.allInstantiations: List<Instantiation> get() = superClassRecursor { instantiations }

/**
 * Given a reactor class, return a list of all its parameters,
 * which includes parameters of base classes that it extends.
 */
val Reactor.allParameters: List<Parameter> get() = superClassRecursor { parameters }

/**
 * Given a reactor class, return a list of all its reactions,
 * which includes reactions of base classes that it extends.
 */
val Reactor.allReactions: List<Reaction> get() = superClassRecursor { reactions }

/**
 * Given a reactor class, return a list of all its state variables,
 * which includes state variables of base classes that it extends.
 */
val Reactor.allStateVars: List<StateVar> get() = superClassRecursor { stateVars }

/**
 * Given a reactor class, return a list of all its  timers,
 * which includes timers of base classes that it extends.
 */
val Reactor.allTimers: List<Timer> get() = superClassRecursor { timers }

private fun <T> Reactor.superClassRecursor(collector: Reactor.() -> List<T>): List<T> =
    superClasses.orEmpty().flatMap { it.toDefinition().collector() } + this.collector()
