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
import org.lflang.generator.cpp.name
import org.lflang.lf.*

// This "record" class duplicates a lot of info that is actuallt already there in the AST. However, I found it most convenient
// to extract the different dependencies and antiDependencies of the reaction into various sets. Particularily it is important
// to deal with the "childReactorInputs" and OUtputs differently. This is because we need to name them differently and also
// code-generate differently to bring them properly into scope. E.g
// reaction(childReactor.out) {==} should reference the `out` port with `childReactor.out`. So some tricks are needed.
class ReactionInfo {
    val portInputs = mutableListOf<Port>()
    val portOutputs = mutableListOf<Port>()
    val timerInputs = mutableListOf<Timer>()
    val builtinInputs = mutableListOf<BuiltinTriggerRef>()
    val childReactorInputs = mutableListOf<Pair<Port, Instantiation>>()
    val childReactorOutputs = mutableListOf<Pair<Port, Instantiation>>()
    val childReactorToInputMap = mutableMapOf<Instantiation, MutableList<Port>>()
    val childReactorToOutputMap = mutableMapOf<Instantiation, MutableList<Port>>()
}

class ChiselReactionGenerator(
    private val reactor: Reactor,
) {
    val reactionInfos = mutableMapOf<Reaction, ReactionInfo>()

    fun generateDeclarations(): String =
        reactor.reactions.joinToString(separator = "\n", postfix = "\n") {
            reactionInfos.putIfAbsent(it, ReactionInfo())
            "${generateDeclaration(it)}".trimMargin()
        }

    fun generateDefinitions(): String =
        reactor.reactions.joinToString(separator = "\n", prefix = "// Reaction definitions\n") {
            "${generateDefinition(it)}".trimMargin()
        }

    fun generatePrecedenceConstraints(): String =
        if (reactor.reactions.size > 1) {
            val builder = StringBuilder()
            builder.appendLine("// Generate precedence constraints connection between the reactions")
            for (r in reactor.reactions) {
                if (r == reactor.reactions.first())
                    builder.append(r.getInstanceName)
                else
                    builder.append(" > ${r.getInstanceName}")
            }
            builder.toString()
        } else {
            ""
        }

    private fun generateDefinition(r: Reaction): String =
        """
            val ${r.getInstanceName} = Module(new ${r.getClassName}(${generateReactionConfig(r)}))
            reactions += ${r.getInstanceName}
        """.trimIndent()

    private fun generateReactionConfig(r: Reaction): String {
        val nPrecedenceInPorts = if (r.indexInContainer >= 1) 1 else 0
        val nPrecedenceOutPorts = if (r.indexInContainer < r.containingReactor.reactions.size - 1) 1 else 0
        return "ReactionConfig(nPrecedenceIn = $nPrecedenceInPorts, nPrecedenceOut = $nPrecedenceOutPorts)"
    }

    private fun generateClassDefinition(r: Reaction): String =
        "class ${r.getClassName}(c: ReactionConfig) extends Reaction(c)"

    private fun generateInputPortFromChildIO(r: Reaction, child: Instantiation, p: Port): String {
        // Store all child reactors and the ports in a dictionary so we can bring them into scope
        // properly later on.
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()

        if (rInfo.childReactorToInputMap.containsKey(child)) {
            rInfo.childReactorToInputMap[child]?.add(p)
        } else {
            rInfo.childReactorToInputMap[child] = mutableListOf(p)
        }
        rInfo.childReactorInputs.add(Pair(p, child))

        return "val ${child.name}__${p.name} = new EventReadMaster(${p.getDataType}, ${p.getTokenType})"
    }
    private fun generateInputPortIO(r: Reaction, p: Port): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        rInfo.portInputs.add(p)
        return "val ${p.getName} = new EventReadMaster(${p.getDataType}, ${p.getTokenType})"
    }

    private fun generateOutputPortIO(r: Reaction, p: Port): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        rInfo.portOutputs.add(p)
        return "val ${p.getName} = new EventWriteMaster(${p.getDataType}, ${p.getTokenType})"
    }

    private fun generateTimerIO(r: Reaction,  t: Timer): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        rInfo.timerInputs.add(t)
        return "val ${t.name} = new EventReadMaster(${t.getDataType}, ${t.getTokenType})"
    }

    private fun generateBuiltinTriggerIO(r: Reaction, t: BuiltinTriggerRef): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        rInfo.builtinInputs.add(t)

        if (t.type == BuiltinTrigger.STARTUP) {
            return "val startup = new EventReadMaster(UInt(0.W), new PureToken)"
        } else if (t.type == BuiltinTrigger.SHUTDOWN) {
            return "val shutdown = new EventReadMaster(UInt(0.W), new PureToken)"
        } else {
            require(false)
            return ""
        }
    }

    private fun generatePortTriggerFromChildIOs(r: Reaction): String =
        r.triggers.filter{it is VarRef}.map{it as VarRef}.filter{it.container is Instantiation}.filter{it.variable is Port}.joinToString(separator = "\n", prefix = "// Port Triggers \n", postfix = "\n") {
            generateInputPortFromChildIO(r, it.container as Instantiation, it.variable as Port)
        }
    private fun generatePortTriggerIOs(r: Reaction): String =
        r.triggers.filter{it is VarRef}.map{it as VarRef}.filterNot{it.container is Instantiation}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Port Triggers \n", postfix = "\n") { generateInputPortIO(r, it) }
    private fun generateBuiltinTriggerIOs(r: Reaction): String =
        r.triggers.filter{it is BuiltinTriggerRef}.map{it as BuiltinTriggerRef}.joinToString(separator = "\n", prefix = "// Builtin triggers\n", postfix = "\n") { generateBuiltinTriggerIO(r, it) }

    private fun generateSourceIOs(r: Reaction): String =
        r.sources.filter{it is VarRef}.map{it as VarRef}.filterNot{it.container is Instantiation}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Port Sources \n", postfix = "\n") { generateInputPortIO(r, it) }

    private fun generateEffectIOs(r: Reaction): String =
        r.effects.filter{it is VarRef}.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Port Effects \n", postfix = "\n") { generateOutputPortIO(r, it) }

    private fun generateTimerIOs(r: Reaction): String =
        r.triggers.filter{it is VarRef}.map{it as VarRef}.map{it.variable}.filterIsInstance<Timer>().joinToString(separator = "\n", prefix = "// Timers \n", postfix = "\n") { generateTimerIO(r, it) }

    private fun generateReactionIOClass(r: Reaction): String = with(PrependOperator) {
        """
            |// IO definition
            |class ${r.getIOClassName} extends ReactionIO {
         ${"|  "..generatePortTriggerIOs(r)}
         ${"|  "..generatePortTriggerFromChildIOs(r)}
         ${"|  "..generateSourceIOs(r)}
         ${"|  "..generateEffectIOs(r)}
         ${"|  "..generateTimerIOs(r)}
         ${"|  "..generateBuiltinTriggerIOs(r)}
            |}
            |
        """.trimMargin()
    }

    private fun generateReactionIODefinition(r: Reaction): String =
        "val io = IO(new ${r.getIOClassName})\n"

    private fun generatePortSeqs(r: Reaction): String =
        """
            ${generateTriggerSeq(r)}
            ${generateAntiDependencySeq(r)}
        """.trimIndent()

    private fun generateTriggerSeq(r: Reaction): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        return  (rInfo.portInputs).joinToString( ",", "override val triggers = Seq("){ "io.${it.name}"} +
                (rInfo.timerInputs).joinToString(separator = ",") {"io.${it.name}"} +
                (rInfo.builtinInputs).joinToString(separator = ",") {"io.${it.name}"} +
                (rInfo.childReactorInputs).joinToString(separator = ",", postfix = ")") {"io.${getChildPortName(it.second, it.first)}"}

    }

    private fun generateAntiDependencySeq(r: Reaction): String =
        (r.effects).joinToString( ",", "override val antiDependencies = Seq(", ")"){ "io.${it.name.replace(".", "__")}" }

    private fun generateIOInScope(r: Reaction): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        return (rInfo.portInputs + rInfo.portOutputs + rInfo.timerInputs).joinToString(separator = "\n", prefix = "// Bring IO into scope \n", postfix = "\n")
            { "val ${it.name} = io.${it.name}" } +
                rInfo.builtinInputs.joinToString(separator = "\n", postfix = "\n") {"val ${it.name} = io.${it.name}"} +
            generatePortsFromChildrenInScope(r) +
            reactor.stateVars.joinToString(separator = "\n", postfix = "\n") {"val ${it.name} = stateIO.${it.name}"}
    }

    private fun generatePortsFromChildrenInScope(r: Reaction): String {
        val rInfo = reactionInfos[r] ?: throw NoSuchElementException()
        return  generateInputPortsFromChildrenInScope(rInfo.childReactorToInputMap) +
                generateOutputPortsFromChildrenInScope(rInfo.childReactorToOutputMap)
    }

    private fun generateInputPortsFromChildrenInScope(portMap: MutableMap<Instantiation, MutableList<Port>>): String {
        val builder = StringBuilder()
        for (child in portMap.keys) {
            builder.appendLine("// Bring ports from child reactor `${child.name}` into scope")
            builder.appendLine("object ${child.name} {")
            for (port in portMap[child]!!) {
                builder.appendLine("  def ${port.name} = io.${getChildPortName(child,port)}")
            }
            builder.appendLine("}")
        }
        return builder.toString()
    }
    private fun generateOutputPortsFromChildrenInScope(portMap: MutableMap<Instantiation, MutableList<Port>>): String {
        return ""
    }

    private fun generateInstance(r: Reaction): String =
        "val ${r.getInstanceName} = Module(new ${r.getClassName}(${generateReactionConfig(r)})"

    private fun generateReactionBody(reaction: Reaction): String {return with(PrependOperator) {
        """
            |// Function wrapping user-written reaction-body
            |def reactionBody(): Unit = {
            |   // Everything below this lines are copied directly from the user-reactions
            |   //------------------------------------------------------------------------------------
        ${" |  "..reaction.code.toText()}
            |}
        """.trimMargin() }
    }

    private fun generateStateIOClass(r: Reaction): String {
        val states = reactor.stateVars.joinToString("\n") {
            "val ${it.name} = new StateReadWriteMaster(${it.getDataType}, ${it.getTokenType})"
        }

        val stateDriveDefs = reactor.stateVars.joinToString("\n") {
            "${it.name}.driveDefaults()"
        }

        return """
            class StateIO extends ReactionStateIO {
                ${states}
                override def driveDefaults(): Unit = {
                    ${stateDriveDefs}
                }
            }
        """.trimIndent()
    }
    private fun generateStateIODefinition(r: Reaction): String =
        "val stateIO = IO(new StateIO())"

    private fun generateDeclaration(r: Reaction): String = with(PrependOperator) {
        """
            |${generateClassDefinition(r)} {
            |  // Bring the reacition API (lf_time_logical() etc) into scope
            |  import ReactionApi._
         ${"|  "..generateReactionIOClass(r)}
         ${"|  "..generateReactionIODefinition(r)}
         ${"|  "..generateStateIOClass(r)}
         ${"|  "..generateStateIODefinition(r)}
         ${"|  "..generatePortSeqs(r)}
         ${"|  "..generateIOInScope(r)}
         ${"|  "..generateReactionBody(r)}
            |   // Finally reactionMain is called which organizes when to 
            |   // trigger the reactionBody and more.
            |   reactionMain()
            | }
            |
        """.trimIndent()
    }

}