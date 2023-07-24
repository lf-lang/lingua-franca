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

    val connectionObjects: MutableSet<String> = mutableSetOf()
    val postIODeclarations: StringBuilder = StringBuilder()

    fun hasInwardPassThroughConnection(input: Input): Boolean = connectionObjects.contains(input.getInwardConnName)

    fun generateDeclarationsPostIO(): String = postIODeclarations.toString()

    fun generateDeclarations(): String =
        reactor.connections.joinToString("\n","// Connections \n", postfix = "\n") {  generateConnection(it)} +
        reactor.timers.joinToString("\n", "// Timer-reaction connections\n", postfix = "\n") {generateTimerConnection(it)} +
        """
            // Startup trigger -> reaction connections
            ${generateStartupTriggerConnection()}
            // Shutdown trigger -> reaction connections
            ${generateShutdownTriggerConnection()}
            
        """.trimIndent() +
        reactor.reactions.joinToString("\n", "// Child Reactor <-> Reactions \n", postfix = "\n") {generateChildReactorReactionConnection(it)} +
        reactor.instantiations.joinToString("\n", "//  Connections between child reactors\n", postfix = "\n") {generateContainedConnection(it)}

    // If the given reaction is triggered by a child reactors output. Or if it writes to the input port of a child reactor
    // Generate the connection object needed for this.
    private fun generateChildReactorReactionConnection(r: Reaction): String {
        val builder = StringBuilder()
        for (input in r.sources + r.triggers) {
            if (input is VarRef) {
                if (input.container is Instantiation) {
                    require(input.variable is Output)
                    // Reaction has trigger/source which is the output port of a child reactor
                    builder.appendLine(generateChildReactorToReactionConnection(r, input))
                }
            }
        }
        for (output in r.effects) {
            if (output is VarRef) {
                if (output.container is Instantiation) {
                    require(output.variable is Input)
                    // This reaction has an effect which is the input port of a child reactor
                    builder.appendLine(generateReactionToChildReactorConnection(r, output))
                }
            }
        }

        return builder.toString()
    }

    private fun generateReactionToChildReactorConnection(r: Reaction, effect: VarRef): String {
        val builder = StringBuilder()
        val effectPort = effect.variable as Input
        val effectParent = effect.container as Instantiation

        if (!connectionObjects.contains(effectPort.getConnName)) {
            builder.appendLine("""
                val ${effectPort.getConnName} = ${effectPort.getConnectionFactory}
                ${effectPort.getConnName} << ${r.getInstanceName}.io.${getChildPortName(effectParent, effectPort)}
                """.trimIndent()
            )
            postIODeclarations.appendLine("${effectPort.getConnName}.construct()")
        }
        builder.appendLine("${effectPort.getConnName} >> ${effectParent.name}.io.${effectPort.name}")
        return builder.toString()
    }
    private fun generateChildReactorToReactionConnection(r: Reaction, trig: VarRef): String {
        val builder = StringBuilder()
        val trigPort = trig.variable as Output
        val trigParent = trig.container as Instantiation

        if (!connectionObjects.contains(trigPort.getConnName)) {
            builder.appendLine("""
                val ${trigPort.getConnName} = ${trigPort.getConnectionFactory}
                ${trigPort.getConnName} << ${trigParent.name}.io.${trigPort.name}
                """.trimIndent()
            )
            postIODeclarations.appendLine("${trigPort.getConnName}.construct()")
        }
        builder.appendLine("${trigPort.getConnName} >> ${r.getInstanceName}.io.${getChildPortName(trigParent, trigPort)}")
        return builder.toString()
    }

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
                require(false)
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
                require(false)
//                builder.append(generateContainedToReactionConnection(triggeredReactions, output, c))
            }
        }
        return builder.toString()
    }

    // Generate code for this connection object. The connection object
    private fun generateConnection(c: Connection): String {
        if (c.isIterated) {
            return generateConnectionIterated(c)
        } else {
            return generateConnectionNormal(c)
        }
    }

    private fun generateConnectionIterated(c: Connection): String {
        assert(c.leftPorts.size==1)
        val builder = StringBuilder()
        val lhs = c.leftPorts.get(0)
        assert(lhs.variable is Port)
        val lhsPort = lhs.variable as Port

        for (rhs in c.rightPorts) {
            assert(rhs.variable is Port)
            val rhsPort = rhs.variable as Port
            if (lhsPort.isInput) {
                builder.appendLine(generatePassthroughInputConnections(rhs, lhs))
            } else if (!rhsPort.isInput) {
                builder.appendLine(generatePassthroughOutputConnections(rhs, lhs))
            } else {
                builder.appendLine(generateContainedReactorConnections(lhs, rhs))
            }
        }
        return builder.toString()
    }

    private fun generateConnectionNormal(c: Connection): String {
        val builder = StringBuilder()

        for (i in 0 until c.leftPorts.size) {
            if (i >= c.rightPorts.size) {
                return builder.toString();
            }

            val rhs = c.rightPorts[i]
            val lhs = c.leftPorts[i]
            assert(lhs.variable is Port)
            assert(rhs.variable is Port)

            val lhsPort = lhs.variable as Port
            val rhsPort = rhs.variable as Port
            if (lhsPort.isInput) {
                // Input port of parent driving input ports of child
                builder.appendLine(generatePassthroughInputConnections(lhs, rhs))
            } else if (!rhsPort.isInput)  {
                builder.appendLine(generatePassthroughOutputConnections(lhs, rhs))
            } else {
                // Output port of container driving
                builder.appendLine(generateContainedReactorConnections(lhs, rhs))
            }
        }
        return builder.toString()
    }

    private fun generatePassthroughInputConnections(lhs: VarRef, rhs: VarRef): String {
        val builder = StringBuilder()
        val lhsPort = lhs.variable as Input

        if (!connectionObjects.contains(lhsPort.getInwardConnName)) {
            builder.appendLine("// Connection factory for connection from input port to child reactor")
            builder.appendLine("val ${lhsPort.getInwardConnName} = ${lhsPort.getInwardConnectionFactory}")
            connectionObjects.add(lhsPort.getInwardConnName)
            postIODeclarations.appendLine("${lhsPort.getInwardConnName} << io.${lhsPort.name}")
            postIODeclarations.appendLine("${lhsPort.getInwardConnName}.construct()")
        }

        val rhsPort = rhs.variable as Port
        val rhsParent = rhs.container as Instantiation
        builder.appendLine("${lhsPort.getInwardConnName} >> ${rhsParent.name}.io.${rhsPort.name}")

        return builder.toString()
    }

    private fun generatePassthroughOutputConnections(lhs: VarRef, rhs: VarRef): String {
        val builder = StringBuilder()
        val lhsPort = lhs.variable as Output
        val rhsPort = rhs.variable as Output
        val lhsParent = lhs.container as Instantiation
        builder.appendLine("${rhsPort.name} << ${lhsParent.name}.io.${lhsPort.name}")
        return builder.toString()
    }

    private fun generateContainedReactorConnections(lhs: VarRef, rhs: VarRef): String {
        val builder = StringBuilder()
        val lhsParent = lhs.container as Instantiation
        val lhsPort= lhs.variable as Port
        if (!connectionObjects.contains(lhs.getConnectionName)) {
           builder.appendLine("""
            // Connection factory for child reactor -> child reactor
            val ${lhs.getConnectionName} = ${lhsPort.getConnectionFactory}
            ${lhs.getConnectionName} << ${lhsParent.name}.io.${lhsPort.name}
            """.trimIndent()
           )
            // Add the construction of the connection object to the postIO declarations
            postIODeclarations.appendLine("${lhs.getConnectionName}.construct()")
            connectionObjects.add(lhs.getConnectionName)
        }
        val rhsPort = rhs.variable as Port
        val rhsParent = rhs.container as Instantiation
        builder.appendLine("${lhs.getConnectionName} >> ${rhsParent.name}.io.${rhsPort.name}")
        return builder.toString()
    }

    private fun generateReactionToContainedConnection(reactions: List<Reaction>, input: Input, contained: Instantiation): String {
        val builder = StringBuilder()
        val pName = "${input}"
        builder.append("""
            val $pName = new OutputPort(new OutputPortConfig(${input.getDataType}, ${input.getTokenType}))
            outPorts += $pName
        """.trimIndent())

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

    private fun generateTimerConnection(t: Timer): String {
        val triggeredReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (trig in r.triggers) {
                if (trig is VarRef && trig.name == t.name) {
                    triggeredReactions += r
                }
            }
        }
        val reactionConns = triggeredReactions.joinToString("\n") {"${t.name}.declareTriggeredReaction(${it.getInstanceName}.${t.name})"}

        return  """
            ${reactionConns}
        """.trimIndent()
    }

    private fun generateStartupTriggerConnection(): String {
        val triggeredReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (trig in r.triggers) {
                if (trig is BuiltinTriggerRef && trig.type == BuiltinTrigger.STARTUP) {
                    triggeredReactions += r
                }
            }
        }
        val reactionConns = triggeredReactions.joinToString("\n") {"startupTrigger.declareTriggeredReaction(${it.getInstanceName}.startup)"}

        return  """
            ${reactionConns}
        """.trimIndent()
    }

    private fun generateShutdownTriggerConnection(): String {
        val triggeredReactions = mutableListOf<Reaction>()
        for (r in reactor.reactions) {
            for (trig in r.triggers) {
                if (trig is BuiltinTriggerRef && trig.type == BuiltinTrigger.SHUTDOWN) {
                    triggeredReactions += r
                }
            }
        }
        val reactionConns = triggeredReactions.joinToString("\n") {"shutdownTrigger.declareTriggeredReaction(${it.getInstanceName}.shutdown)"}

        return  """
            ${reactionConns}
        """.trimIndent()
    }
}
