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

class ChiselConnectionGenerator(private val reactor: Reactor) {

    fun generateDeclarations(preIO: Boolean): String =
        reactor.connections.joinToString("\n","//Contained reactor-reactor connections", postfix = "\n") {  generateConnection(it, preIO)} +
        if (preIO) {
            reactor.timers.joinToString("\n", "// Timer-reaction connections\n", postfix = "\n") {generateTimerConnection(it)} +
            reactor.instantiations.joinToString("\n", "// Contained reactor \n", postfix = "\n") {generateContainedConnection(it)}
        } else ""



    private fun generateContainedConnection(c: Instantiation): String {
        val builder = StringBuilder()
        for (input in c.reactor.inputs) {
            val writingReactions = mutableListOf<Reaction>()
            for (r in reactor.reactions) {
                for (e in r.effects) {
                    if (e == input) {
                        writingReactions += r
                    }
                }
            }
            if (writingReactions.isNotEmpty()) {
                builder.append(generateReactionToContainedConnection(writingReactions, input, c))
            }
        }

        for (output in c.reactor.outputs) {
            val triggeredReactions = mutableListOf<Reaction>()
            for (r in reactor.reactions) {
                for (trig in r.sources + r.triggers) {
                    if (trig == output) {
                        triggeredReactions += r
                    }
                }
            }
            if (triggeredReactions.isNotEmpty()) {
                builder.append(generateContainedToReactionConnection(triggeredReactions, output, c))
            }
        }
        return builder.toString()
    }

    private fun generateConnection(lhs: Port, rhs: List<Port>, preIO: Boolean): String {
        val builder = StringBuilder()
        return builder.toString();
    }
    private fun generateConnection(c: Connection, preIO: Boolean): String {
        if (c.hasMultipleConnections) {
            assert(false)
        }
        assert(c.leftPorts.size == 1)

        val lhs = c.leftPorts[0]

        if (lhs is Port) {
            val portInput = lhs as Port
            if (portInput.isInput) {
                // Input port of parent driving input ports of child
                generatePassthroughInputConnections(c, preIO)
            } else {
                // Output port of container driving
                generateContainedReactorConnections(c, preIO)
            }

        } else {
            // Left side is not a port?
            assert(false)
        }

        return "Error";
    }

    private fun generatePassthroughInputConnections(c: Connection, preIO: Boolean): String {
        val upstream = c.leftPorts[0]
        val connName = "ptConn_${upstream.name}"

        val downstreamConns = c.rightPorts.joinToString("\n", postfix = "\n") {
            "${connName}.declareDownstream(${it.name})"
        }
        return """
            val ptConn_${upstream.name} = new InputPortPassthroughBuilder(defData, defToken)
            $downstreamConns
        """.trimIndent()
    }

    private fun generateContainedReactorConnections(c: Connection, preIO: Boolean): String {
        return """
            val 
        """.trimIndent()
    }

    private fun generateReactionToContainedConnection(reactions: List<Reaction>, input: Input, contained: Instantiation): String {
        val builder = StringBuilder()
        val pName = "${input}"
        builder.appendLine("val $pName = new OutputPort(new OutputPortConfig(${input.getDataType}, ${input.getTokenType}))")

        for (r in reactions) {
            builder.appendLine("$pName << $r.io.${input.name}")
        }

        builder.append("""
            val ${input.getConnFuncName} = ${input.getConnFunc}
            val ${input.getConnName} = new ConnectionBuilder(${input.getConnFuncName}, ${input.getDataType}, ${input.getTokenType})
            ${pName} >> ${input.getConnName} 
            ${input.getConnName} >> ${contained.name}.io.${input.name}
            ${input.getConnName}.construct()
        """.trimIndent())

        return builder.toString()
    }
    private fun generateContainedToReactionConnection(reactions: List<Reaction>, output: Output, contained: Instantiation) = ""



    private fun generateTimerConnection(t: Timer): String {
        val conn_name = "conn_${t.name}"
        val triggeredReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (trig in r.triggers) {
                if (trig == t)
                    triggeredReactions += r
            }
        }
        val reactionConns = triggeredReactions.joinToString("\n", postfix = "\n") {"${conn_name} >> ${it.name}"}

        return  """
            val ${conn_name} = new PureConnectionBuilder()
            // Have timer drive the connection
            $conn_name << ${t.name}
            ${reactionConns}
            ${conn_name}.construct()
        """.trimIndent()
    }
}
