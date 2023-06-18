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
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.cpp.name
import org.lflang.lf.*

class ChiselReactionGenerator(
    private val reactor: Reactor,
) {

    fun generateDeclarations(): String =
        reactor.reactions.joinToString(separator = "\n", postfix = "\n") {
            "${generateDeclaration(it)}".trimMargin()
        }


    fun generateDefinitions(): String =
        reactor.reactions.joinToString(separator = "\n", prefix = "// Reaction definitions\n") {
            "${generateDefinition(it)}".trimMargin()
        }

    fun generatePrecedenceConstraints(): String =
        if (reactor.reactions.size > 1) {
            val builder = StringBuilder()
            for (r in reactor.reactions) {
                if (r == reactor.reactions.first())
                    builder.append(r.name)
                else
                    builder.append("> ${r.name}")
            }
            builder.toString()
        } else {
            ""
        }
    private fun generateDefinition(r: Reaction): String =
        "val ${r.getInstanceName} = Module(new ${r.getClassName}(${generateReactionConfig(r)}))"

    private fun generateReactionConfig(r: Reaction): String {
        val nPrecedenceInPorts = if (r.indexInContainer > 1) 1 else 0
        val nPrecedenceOutPorts = if (r.indexInContainer < r.containingReactor.reactions.size-1) 1 else 0
        return "ReactionConfig(nPrecedenceIn = $nPrecedenceInPorts, nPrecedenceOut = $nPrecedenceOutPorts)"
    }
    private fun generateClassDefinition(r: Reaction): String =
        "class ${r.getClassName}(c: ReactionConfig) extends Reaction(c)"

    private fun generateInputPortIO(p: Port): String =
        "val ${p.name} = new EventReadMaster(${p.getDataType}, ${p.getTokenType})"

    private fun generateOutputPortIO(p: Port): String =
        "val ${p.name} = new EventWriteMaster(${p.getDataType}, ${p.getTokenType})"

    private fun generateTimerIO(t: Timer): String =
        "val ${t.name} = new EventReadMaster(${t.getDataType}, ${t.getTokenType})"

    private fun generateTriggerIOs(r: Reaction): String =
        r.triggers.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Triggers \n", postfix = "\n") { generateInputPortIO(it) }

    private fun generateSourceIOs(r: Reaction): String =
        r.sources.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Sources \n", postfix = "\n") { generateInputPortIO(it) }

    private fun generateEffectIOs(r: Reaction): String =
        r.effects.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Effects \n", postfix = "\n") { generateOutputPortIO(it) }

    private fun generateTimerIOs(r: Reaction): String =
        r.triggers.map{it as VarRef}.map{it.variable}.filterIsInstance<Timer>().joinToString(separator = "\n", prefix = "// Timers \n", postfix = "\n") { generateTimerIO(it) }

    private fun generateReactionIOClass(r: Reaction): String = with(PrependOperator) {
        """
            |// IO definition
            |class ${r.getIOClassName} extends ReactionIO {
         ${"|  "..generateTriggerIOs(r)}
         ${"|  "..generateSourceIOs(r)}
         ${"|  "..generateEffectIOs(r)}
         ${"|  "..generateTimerIOs(r)}
            |}
            |
        """.trimMargin()

    }

    private fun generateReactionIODefinition(r: Reaction): String =
        "val io = IO(new ${r.getIOClassName})\n"

    private fun generateTriggerSeq(r: Reaction): String =
        (r.triggers + r.sources).joinToString( ",", "override val triggers = Seq(", ")"){ "io.${it.name}" }

    private fun generateIOInScope(r: Reaction): String =
        (r.triggers + r.sources + r.effects).joinToString(separator = "\n", prefix = "// Bring IO into scope \n", postfix = "\n")
        { "val ${it.name} = io.${it.name}" } +
                reactor.stateVars.joinToString(separator = "\n", postfix = "\n") {"val ${it.name} = stateIO.${it.name}"}

    private fun generateInstance(r: Reaction): String =
        "val ${r.getInstanceName} = Module(new ${r.getClassName}(${generateReactionConfig(r)})"

    private fun generateReactionBody(reaction: Reaction): String =
        """
            def reactionBody(): Unit = {
               ${reaction.code.toText()}
             }
        """.trimIndent()

    private fun generateStateIOClass(r: Reaction): String {
        val states = reactor.stateVars.joinToString("\n") {
            "val ${it.name} = new StateReadWriteMaster(${it.getDataType}, ${it.getTokenType})"
        }
        return """
            class StateIO extends ReactionStateIO {
                ${states}
            }
        """.trimIndent()
    }
    private fun generateStateIODefinition(r: Reaction): String =
        "val stateIO = IO(new StateIO())"

    private fun generateDeclaration(r: Reaction): String = with(PrependOperator) {
        """
            |${generateClassDefinition(r)} {
            |  import ReactionApi._
         ${"|  "..generateReactionIOClass(r)}
         ${"|  "..generateReactionIODefinition(r)}
         ${"|  "..generateStateIOClass(r)}
         ${"|  "..generateStateIODefinition(r)}
         ${"|  "..generateTriggerSeq(r)}
         ${"|  "..generateIOInScope(r)}
         ${"|  "..generateReactionBody(r)}
            |   reactionMain()
            | }
            |
        """.trimIndent()

    }
}