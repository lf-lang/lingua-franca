/*************
 * Copyright (c) 2021, TU Dresden.

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
 ***************/

package org.lflang.generator.cpp

import org.lflang.inferredType
import org.lflang.isInitialized
import org.lflang.isOfTimeType
import org.lflang.lf.ParameterReference
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar

/** A C++ code generator for state variables */
class CppStateGenerator(private val reactor: Reactor) {

    /**
     * Create a list of state initializers in target code.
     *
     * TODO This is redundant to ExpressionGenerator.getInitializerList
     */
    private fun getInitializerList(state: StateVar) = state.init.map {
        when {
            it is ParameterReference -> it.parameter.name
            state.isOfTimeType       -> it.toTime()
            else                     -> it.toCppCode()
        }
    }

    private fun generateInitializer(state: StateVar): String =
        if (state.parens.isNullOrEmpty())
            "${state.name}{${getInitializerList(state).joinToString(separator = ", ")}}"
        else
            "${state.name}(${getInitializerList(state).joinToString(separator = ", ")})"


    /** Get all state declarations */
    fun generateDeclarations() =
        reactor.stateVars.joinToString("\n", "// state variable\n", "\n") { "${it.inferredType.cppType} ${it.name};" }

    /** Get all timer initializers */
    fun generateInitializers(): String = reactor.stateVars.filter { it.isInitialized }
        .joinToString(separator = "\n", prefix = "// state variables\n") { ", ${generateInitializer(it)}" }
}
