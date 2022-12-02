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

import org.lflang.generator.PrependOperator
import org.lflang.lf.Reactor

/** A code generator for the C++ constructor of a reactor class */
class CppConstructorGenerator(
    private val reactor: Reactor,
    private val parameters: CppParameterGenerator,
    private val state: CppStateGenerator,
    private val instances: CppInstanceGenerator,
    private val timers: CppTimerGenerator,
    private val actions: CppActionGenerator,
    private val ports: CppPortGenerator,
    private val reactions: CppReactionGenerator,
) {

    /**
     * Constructor argument that provides a reference to the next higher level
     *
     * For the main reactor, the next higher level is the environment. For all other reactors, it is the containing reactor.
     */
    private val environmentOrContainer =
        if (reactor.isMain) "reactor::Environment* environment" else "reactor::Reactor* container"

    private val outerSignature: String =
         "${reactor.name}(const std::string& name, $environmentOrContainer, const Parameters& parameters)"

    private val innerSignature: String = "Inner(reactor::Reactor* reactor, [[maybe_unused]] const Parameters& parameters)"

    /** Get the constructor declaration of the outer reactor class */
    fun generateOuterDeclaration() = "$outerSignature;"

    /** Get the constructor definition of the outer reactor class */
    fun generateOuterDefinition(): String {
        return with(PrependOperator) {
            """
                |${reactor.templateLine}
                |${reactor.templateName}::$outerSignature
                |  : reactor::Reactor(name, ${if (reactor.isMain) "environment" else "container"})
            ${" |  "..instances.generateInitializers()}
            ${" |  "..timers.generateInitializers()}
            ${" |  "..actions.generateInitializers()}
            ${" |  "..reactions.generateReactionViewInitializers()}
                |  , __lf_inner(this, parameters)
                |{
            ${" |  "..ports.generateConstructorInitializers()}
            ${" |  "..instances.generateConstructorInitializers()}
            ${" |  "..reactions.generateReactionViewConstructorInitializers()}
                |}
            """.trimMargin()
        }
    }

    /** Get the constructor declaration of the inner reactor class */
    fun generateInnerDeclaration() = "$innerSignature;"

    fun generateInnerDefinition(): String {
        return with(PrependOperator) {
            """
                |${reactor.templateLine}
                |${reactor.templateName}::Inner::$innerSignature
                |  : LFScope(reactor)
            ${" |  "..parameters.generateInitializers()}
            ${" |  "..state.generateInitializers()}
                |{}
            """.trimMargin()
        }
    }
}
