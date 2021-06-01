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

import org.lflang.lf.*

/**
 * A code generator for the assemble() method of a C++ reactor class
 *
 * The assemble method is called once during initialization by the reactor runtime. It is
 * responsible for declaring all triggers, dependencies and effects (antidependencies) of reactions.
 * It is also responsible for creating all connections within the reactor.
 */
class CppAssembleMethodGenerator(private val reactor: Reactor) {

    private fun declareTrigger(reaction: Reaction, trigger: TriggerRef): String {
        // check if the trigger is a multiport
        if (trigger is VarRef && trigger.variable is Port) {
            val port = trigger.variable as Port
            if (port.widthSpec != null) {
                return """
                    for (unsigned i = 0; i < ${trigger.name}.size(); i++) {
                      ${reaction.name}.declare_trigger(&${trigger.name}[i]);
                    }
                """.trimIndent()
            }
        }
        // treat as single trigger otherwise
        return "${reaction.name}.declare_trigger(&${trigger.name});"
    }

    private fun declareDependency(reaction: Reaction, dependency: VarRef): String {
        val variable = dependency.variable
        // check if the dependency is a multiport
        if (variable is Port && variable.widthSpec != null) {
            return """
                for (unsigned i = 0; i < ${dependency.name}.size(); i++) {
                  ${reaction.name}.declare_dependency(&${dependency.name}[i]);
                }
            """.trimIndent()
        }
        // treat as single dependency otherwise
        return "${reaction.name}.declare_dependency(&${dependency.name});"
    }

    private fun declareAntidependency(reaction: Reaction, antidependency: VarRef): String {
        val variable = antidependency.variable
        // check if the dependency is a multiport
        if (variable is Port && variable.widthSpec != null) {
            return """
                for (unsigned i = 0; i < ${antidependency.name}.size(); i++) {
                  ${reaction.name}.declare_antidependency(&${antidependency.name}[i]);
                }
            """.trimIndent()
        }
        // treat as single antidependency otherwise
        return if (variable is Action) "${reaction.name}.declare_scheduable_action(&${antidependency.name});"
        else "${reaction.name}.declare_antidependency(&${antidependency.name});"
    }

    private fun assembleReaction(reaction: Reaction) = with(prependOperator) {
        """
            |// ${reaction.name}
        ${" |"..reaction.triggers.joinToString(separator = "\n") { declareTrigger(reaction, it) }}
        ${" |"..reaction.sources.joinToString(separator = "\n") { declareDependency(reaction, it) }}
        ${" |"..reaction.effects.joinToString(separator = "\n") { declareAntidependency(reaction, it) }}
            |// TODO «IF n.deadline !== null»
            |// TODO «n.name».set_deadline(«n.deadline.delay.targetTime», [this]() { «n.name»_deadline_handler(); });
            |// TODO «ENDIF»
        """.trimMargin()
    }

    private fun declareConnection(c: Connection): String {
        assert(c.leftPorts.size == 1)
        assert(c.rightPorts.size == 1)

        val leftPort = c.leftPorts[0]
        val rightPort = c.rightPorts[0]

        return "${leftPort.name}.bind_to(&${rightPort.name});"

        // TODO Support multiports and banks
        /*val result = StringBuffer()
        var leftPort = c.leftPorts[0]
        var leftPortCount = 1
        // The index will go from zero to mulitportWidth - 1.
        var leftPortIndex = 0
        // FIXME: Support parameterized widths and check for matching widths with parallel connections.
        var leftWidth = leftPort.portWidth(c)
        var leftContainer = leftPort.container
        var rightPortCount = 0
        for (rightPort in c.rightPorts) {
            rightPortCount++
            var rightPortIndex = 0
            val rightContainer = rightPort.container
            val rightWidth = rightPort.portWidth(c)
            while (rightPortIndex < rightWidth) {
                // Figure out how many bindings to do.
                var remainingRightPorts = rightWidth - rightPortIndex
                var remainingLeftPorts = leftWidth - leftPortIndex
                var min = (remainingRightPorts < remainingLeftPorts)?
                remainingRightPorts : remainingLeftPorts
                // If the right or left port is a port in a bank of reactors,
                // then we need to construct the index for the bank.
                // Start with the right port.
                var rightContainerRef = ''
                var rightPortArrayIndex = ''
                if (rightContainer !== null) {
                    if (rightContainer.widthSpec !== null) {
                        // The right port is within a bank of reactors.
                        var rightMultiportWidth = 1
                        if ((rightPort.variable as Port).widthSpec !== null) {
                            // The right port is also a multiport.
                            rightMultiportWidth = calcPortWidth(rightPort.variable as Port)
                            rightPortArrayIndex = '''[(«rightPortIndex» + i) % «rightMultiportWidth»]'''
                        }
                        rightContainerRef = '''«rightContainer.name»[(«rightPortIndex» + i) / «rightMultiportWidth»]->'''
                    } else {
                        rightContainerRef = '''«rightContainer.name»->'''
                        if ((rightPort.variable as Port).widthSpec !== null) {
                            rightPortArrayIndex = '''[«rightPortIndex» + i]'''
                        }
                    }
                } else if ((rightPort.variable as Port).widthSpec !== null) {
                    // The right port is not within a bank of reactors but is a multiport.
                    rightPortArrayIndex = '''[«rightPortIndex» + i]'''
                }
                // Next, do the left port.
                var leftContainerRef = ''
                var leftPortArrayIndex = ''
                if (leftContainer !== null) {
                    if (leftContainer.widthSpec !== null) {
                        // The left port is within a bank of reactors.
                        var leftMultiportWidth = 1
                        if ((leftPort.variable as Port).widthSpec !== null) {
                            // The left port is also a multiport.
                            // FIXME: Does not support parameter values for widths.
                            leftMultiportWidth = calcPortWidth(leftPort.variable as Port)
                            leftPortArrayIndex = '''[(«leftPortIndex» + i) % «leftMultiportWidth»]'''
                        }
                        leftContainerRef = '''«leftContainer.name»[(«leftPortIndex» + i) / «leftMultiportWidth»]->'''
                    } else {
                        leftContainerRef = '''«leftContainer.name»->'''
                        if ((leftPort.variable as Port).widthSpec !== null) {
                            leftPortArrayIndex = '''[«leftPortIndex» + i]'''
                        }
                    }
                } else if ((leftPort.variable as Port).widthSpec !== null) {
                    // The left port is not within a bank of reactors but is a multiport.
                    leftPortArrayIndex = '''[«leftPortIndex» + i]'''
                }
                result.append('''
                    for (unsigned i = 0; i < «min»; i++) {
                    «leftContainerRef»«leftPort.variable.name»«leftPortArrayIndex»
                    .bind_to(&«rightContainerRef»«rightPort.variable.name»«rightPortArrayIndex»);
                }
                ''')
                rightPortIndex += min
                leftPortIndex += min
                if (leftPortIndex == leftPort.portWidth(c)) {
                    if (leftPortCount < c.leftPorts.length) {
                        // Get the next left port. Here we rely on the validator to
                        // have checked that the connection is balanced, which it does only
                        // when widths are given as literal constants.
                        leftPort = c.leftPorts.get(leftPortCount++)
                        leftWidth = leftPort.portWidth(c)
                        leftPortIndex = 0
                        leftContainer = leftPort.container
                    } else {
                        // We have run out of left ports.
                        // If the connection is a broadcast connection,
                        // then start over.
                        if (c.isIterated) {
                            leftPort = c.leftPorts.get(0)
                            leftPortCount = 1
                            leftWidth = leftPort.portWidth(c)
                            leftPortIndex = 0
                            leftContainer = leftPort.container
                        } else if (rightPortCount < c.rightPorts.length || rightPortIndex < rightWidth - 1) {
                            c.reportWarning("More right ports than left ports. Some right ports will be unconnected.")
                        }
                    }
                }
            }
        }
        return result.toString*/
    }

    /**
     * Generate the definition of the reactor's assemble() method
     *
     * The body of this method will declare all triggers, dependencies and antidependencies to the runtime.
     */
    fun generateDefinition() = with(prependOperator) {
        """
            |// TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
            |void ${reactor.templateName}::assemble() {
        ${" |  "..reactor.reactions.joinToString(separator = "\n\n") { assembleReaction(it) }}
        ${" |  "..reactor.connections.joinToString(separator = "\n", prefix = "// connections\n") { declareConnection(it) }}
            |}
        """.trimMargin()
    }
}