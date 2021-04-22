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

import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.lf.*


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
    superClasses.orEmpty().mapNotNull { it.toDefinition()?.collector() }.flatten() + this.collector()

val Parameter.isOfTimeType: Boolean
    get() {
        // Either the type has to be declared as a time.
        if (type?.isTime == true) {
            return true
        }
        // Or it has to be initialized as a proper time with units.
        init?.singleOrNull()
            ?.time
            ?.takeIf { it.unit != TimeUnit.NONE }
            ?.let {
                return true
            }
        // In other words, one can write:
        // - `x:time(0)` -OR- 
        // - `x:(0 msec)`, `x:(0 sec)`, etc.     
        return false
    }

fun <T> List<T>.tail() = subList(1, size)
fun <T> List<T>.headAndTail() = Pair(first(), tail())


fun Code?.toText(): String {
    this ?: return ""
    val node = NodeModelUtils.getNode(this)
    return if (node != null) {
        val str = node.leafNodes
            .joinToString { it.text }.trim()
            .removeSurrounding("{=", "=}")

        if ('\n' in str) str.trimIndent() else str.trim()
    } else if (body != null) {
        // Code must have been added as a simple string.
        body.toString()
    } else {
        ""
    }
}

fun TypeParm.toText(): String =
    if (!literal.isNullOrEmpty()) literal
    else code.toText()

