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
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.*
import kotlin.math.ceil

/**
 * A code generator for the assemble() method of a C++ reactor class
 *
 * The assemble method is called once during initialization by the reactor runtime. It is
 * responsible for declaring all triggers, dependencies and effects (antidependencies) of reactions.
 * It is also responsible for creating all connections within the reactor.
 */
class CppAssembleMethodGenerator(
    private val reactor: Reactor,
    private val portGenerator: CppPortGenerator,
    private val instanceGenerator: CppInstanceGenerator,
) {

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
        return if (variable is Action) "${reaction.name}.declare_schedulable_action(&${antidependency.name});"
        else "${reaction.name}.declare_antidependency(&${antidependency.name});"
    }

    private fun setDeadline(reaction: Reaction): String =
        "${reaction.name}.set_deadline(${reaction.deadline.delay.toTime(true)}, [this]() { ${reaction.name}_deadline_handler(); });"

    private fun assembleReaction(reaction: Reaction) = with(PrependOperator) {
        """
            |// ${reaction.name}
        ${" |"..reaction.triggers.joinToString(separator = "\n") { declareTrigger(reaction, it) }}
        ${" |"..reaction.sources.joinToString(separator = "\n") { declareDependency(reaction, it) }}
        ${" |"..reaction.effects.joinToString(separator = "\n") { declareAntidependency(reaction, it) }}
        ${" |"..if (reaction.deadline != null) setDeadline(reaction) else ""}
        """.trimMargin()
    }

    /** A data class for holding all information that is relevant for reverencing one specific port
     *
     * The port could be a member of a bank instance and it could be an instance of a multiport.
     * Thus, the information in this class includes a bank and port index. If the bank (or port)
     * index is null, then the referenced port is not part of a bank (or multiport).
     */
    private data class PortReference(val port: Port, val portIndex: Int?, val container: Instantiation?, val containerIndex: Int?)

    private fun PortReference.toCode(): String {
        val portRef = if (port.isMultiport) "${port.name}[$portIndex]" else port.name
        return if (container != null) {
            val containerRef = if (container.isBank) "${container.name}[$containerIndex]" else container.name
            "$containerRef->$portRef"
        } else {
            portRef
        }
    }

    /** Get a list of PortReferences for the given list of variables
     *
     * This checks whether the variable refers to a multiport and generated an instance of
     * PortReferrence for each port instance in the multiport. If the port is containe in a
     * multiport, the result includes instances PortReference for each pair of bank and multiport
     * instance.
     */
    private fun enumerateAllPortsFromReferences(references: List<VarRef>): List<PortReference> {
        val ports = mutableListOf<PortReference>()

        for (ref in references) {
            val container = ref.container
            val port = ref.variable as Port
            val bankIndexes =
                if (container?.isBank == true) with(instanceGenerator) { (0 until container.getValidWidth()) }
                else listOf<Int?>(null)
            val portIndexes =
                if (port.isMultiport) with(portGenerator) { (0 until port.getValidWidth()) }
                else listOf<Int?>(null)
            // calculate the Cartesian product af both index lists defined above
            // TODO iterate over banks or ports first?
            val indexPairs = portIndexes.flatMap { portIdx -> bankIndexes.map { bankIdx -> portIdx to bankIdx } }
            ports.addAll(indexPairs.map { PortReference(port, it.first, container, it.second) })
        }
        return ports
    }

    private fun declareConnection(c: Connection): String {
        val lhsPorts = enumerateAllPortsFromReferences(c.leftPorts)
        val rhsPorts = enumerateAllPortsFromReferences(c.rightPorts)

        // If the connection is a broadcast connection, then repeat the lhs ports until it is equal
        // or greater to the number of rhs ports. Otherwise, continue with the unmodified list of lhs
        // ports
        val iteratedLhsPorts = if (c.isIsIterated) {
            val numIterations = ceil(rhsPorts.size.toDouble() / lhsPorts.size.toDouble()).toInt()
            (1..numIterations).flatMap { lhsPorts }
        } else {
            lhsPorts
        }

        // bind each pair of lhs and rhs ports individually
        return (iteratedLhsPorts zip rhsPorts).joinToString("\n") {
            "${it.first.toCode()}.bind_to(&${it.second.toCode()});"
        }
    }

    /**
     * Generate the definition of the reactor's assemble() method
     *
     * The body of this method will declare all triggers, dependencies and antidependencies to the runtime.
     */
    fun generateDefinition() = with(PrependOperator) {
        """
            |${reactor.templateLine}
            |void ${reactor.templateName}::assemble() {
        ${" |  "..reactor.reactions.joinToString(separator = "\n\n") { assembleReaction(it) }}
        ${" |  "..reactor.connections.joinToString(separator = "\n", prefix = "// connections\n") { declareConnection(it) }}
            |}
        """.trimMargin()
    }
}