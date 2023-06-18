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

import org.lflang.ErrorReporter
import org.lflang.generator.PrependOperator
import org.lflang.isGeneric
import org.lflang.lf.Input
import org.lflang.lf.Output
import org.lflang.lf.Reactor
import org.lflang.toText
import org.lflang.toUnixString

/**
 * A C++ code generator that produces a C++ class representing a single reactor
 */
class ChiselReactorGenerator(private val reactor: Reactor, fileConfig: ChiselFileConfig, errorReporter: ErrorReporter) {

    private val state = ChiselStateGenerator(reactor)
    private val instances = ChiselInstanceGenerator(reactor, fileConfig, errorReporter)
    private val timers = ChiselTimerGenerator(reactor)
    private val ports = ChiselPortGenerator(reactor)
    private val reactions = ChiselReactionGenerator(reactor)
    private val connections = ChiselConnectionGenerator(reactor)

    private fun generateIOInput(input: Input): String {
        val nReactionsTriggered = 1 // FIXME: Find this
        val hasPassthrough = false

        if (hasPassthrough) {
            return "val ${input.name} = Vec($nReactionsTriggered + ptConn_${input.name}.width, new EventReadMaster(defData, defToken))"
        } else {
            return "val ${input.name} = Vec($nReactionsTriggered, new EventReadMaster(defData, defToken))"
        }
    }

    private fun generatePlugUnusedFunc(): String = with(PrependOperator) {
        val plugs = reactor.inputs.joinToString("\n") {"${it.name}.driveDefaults()" }
        return """
            def plugUnusedPorts(): Unit = {
                ${plugs}
            }
        """.trimIndent()
    }

    private fun generateIOOutput(output: Output): String =
        "val ${output.name} = new EventWriteMaster(defData, defToken)"
    private fun generateIO(): String = with(PrependOperator) {
            val inputs = reactor.inputs.joinToString("\n"){generateIOInput(it)}
            val outputs = reactor.outputs.joinToString("\n"){generateIOOutput(it)}
            val plugUnusedFunc = generatePlugUnusedFunc()
            return """
                |// The IO declaration of this Reactor
                |class ${reactor.name}IO extends ReactorIO {
             ${"|  "..inputs}
             ${"|  "..outputs}
             ${"|  "..plugUnusedFunc}
                |}
                |// The IO definition of this module
                |val io = IO(new ${reactor.name}IO())
            """.trimMargin()
    }

    fun generateSource() = with(PrependOperator) {
        """
            |package lf.${reactor.name}
            |import chisel3._
            |import chisel3.util._
            |import reactor._
            |///////////////////////////////////////////////////////////////////////////////////////////////////////
            |// Reaction declarations
            |///////////////////////////////////////////////////////////////////////////////////////////////////////
            |${reactions.generateDeclarations()} 
            |
            |///////////////////////////////////////////////////////////////////////////////////////////////////////
            |// Reactor declaration
            |///////////////////////////////////////////////////////////////////////////////////////////////////////
            |class ${reactor.name} extends Reactor {
        ${" |  "..timers.generateDeclarations()}
        ${" |  "..ports.generateDeclarations()}
        ${" |  "..reactions.generateDefinitions()}
        ${" |  "..reactions.generatePrecedenceConstraints()}
        ${" |  "..instances.generateDeclarations()}
        ${" |  "..connections.generateDeclarations(preIO=true)}
        ${" |  "..generateIO()}
        ${" |  "..connections.generateDeclarations(preIO=false)}
            |  val timerIO = connectTimersAndCreateIO()
            |  reactorMain()
            |}
            |
        """.trimMargin()
    }
}



