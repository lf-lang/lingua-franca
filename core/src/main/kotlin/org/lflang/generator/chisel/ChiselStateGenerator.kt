/**
 * @author Erling R. Jellum (erling.r.jellum@ntnu.no)
 *
 * Copyright (c) 2023, The Norwegian University of Science and Technology.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

package org.lflang.generator.chisel

import org.lflang.generator.PrependOperator
import org.lflang.inferredType
import org.lflang.isInitialized
import org.lflang.joinWithLn
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar

/** A C++ code generator for state variables */
class ChiselStateGenerator(private val reactor: Reactor) {

    /** Get all state declarations */
    fun generateDeclarations() =
        reactor.stateVars.joinToString("\n", "// State variables\n", "\n") {generateState(it)}


    private fun generateState(state: StateVar) = with(PrependOperator) {
        """
        |${generateStateDeclaration(state)}
        |${generateStateToReactionConnection(state)}
       """.trimMargin()
    }

    private fun generateStateDeclaration(state: StateVar) =
        """
            val ${state.name} = Module(${state.getStateDecl})
            states += ${state.name}
        """.trimIndent()

    private fun generateStateToReactionConnection(state: StateVar): String {
        val builder = StringBuilder()
        for ((i,r) in reactor.reactions.withIndex()) {
            builder.appendLine("${state.name}.io.ports($i) <> ${r.getInstanceName}.stateIO.${state.name}")
        }
        return builder.toString()
    }

//    /** Get all state initializers */
//    fun generateInitializers(): String =
//        reactor.stateVars.filter { it.isInitialized }
//            .joinWithLn(prefix = "// state variables\n") {
//                ", " + it.name + CppTypes.getCppInitializer(it.init, it.inferredType, disableEquals = true)
//            }
}
