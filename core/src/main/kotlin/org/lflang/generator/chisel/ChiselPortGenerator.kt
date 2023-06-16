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

import org.lflang.*
import org.lflang.generator.cpp.name
import org.lflang.lf.*

class ChiselPortGenerator(private val reactor: Reactor) {

    // FIXME: We need to get data type and token type
    private fun generateInputPortDeclaration(p: Input) =
        """
        val ${p.name} = Module(new InputPort(InputPortConfig(${p.getDataType}, ${p.getTokenType})))
        """.trimIndent()


    private fun generateOutputPortDeclaration(p: Output) =
        "val ${p.name} = Module(new OutputPort(OutputPortConfig(${p.getDataType}, ${p.getTokenType})))"


    fun generateDeclarations() =
        reactor.inputs.joinToString("\n", "// input ports\n", postfix = "\n") { generateInputPortDeclaration(it) } +
        reactor.outputs.joinToString("\n", "// output ports\n", postfix = "\n") { generateOutputPortDeclaration(it) }


    fun generateConnections() =
        reactor.inputs.joinToString("\n", "// input ports\n", postfix = "\n") { generateInputPortConnection(it as Input) } +
        reactor.outputs.joinToString("\n", "// output ports\n", postfix = "\n") { generateOutputPortConnection(it as Output) }

    fun generateInputPortConnection(p: Input): String {
        val triggeredReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (dep in (r.triggers + r.sources)) {
                if (dep == p)
                    triggeredReactions += r
            }
        }
        val reactionConns = triggeredReactions.joinToString("\n", postfix = "\n") {"${p.name} >> ${it.name}"}

        return """
            ${p.name} << io.${p.name}
            $reactionConns
        """.trimIndent()
    }

    fun generateOutputPortConnection(p: Output): String {
        val writingReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (antiDep in (r.effects)) {
                if (antiDep == p)
                    writingReactions += r
            }
        }
        val reactionConns = writingReactions.joinToString("\n", postfix = "\n") {"${p.name} << ${it.name}"}

        return """
            ${p.name} >> io.${p.name}
            $reactionConns
        """.trimIndent()
    }
}
