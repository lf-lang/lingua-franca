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

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.CppConnectionGenerator.Companion.name
import org.lflang.generator.cpp.CppConnectionGenerator.Companion.requiresConnectionClass
import org.lflang.generator.cpp.CppPortGenerator.Companion.dataType
import org.lflang.lf.Action
import org.lflang.lf.Connection
import org.lflang.lf.ParameterReference
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef

/**
 * A code generator for the assemble() method of a C++ reactor class
 *
 * The assemble method is called once during initialization by the reactor runtime. It is
 * responsible for declaring all triggers, dependencies and effects (antidependencies) of reactions.
 * It is also responsible for creating all connections within the reactor.
 */
class CppAssembleMethodGenerator(private val reactor: Reactor) {

    private fun iterateOverAllPortsAndApply(
        varRef: VarRef,
        generateCode: (String) -> String
    ): String {
        val port = varRef.variable as Port
        val container = varRef.container
        return with(PrependOperator) {
            if (port.isMultiport) {
                if (container?.isBank == true) {
                    if (varRef.isInterleaved) {
                        """
                            |for (size_t __lf_port_idx = 0; __lf_port_idx < ${container.name}[0]->${port.name}.size(); __lf_port_idx++) {
                            |  for (auto& __lf_instance : ${container.name}) {
                        ${" |    "..generateCode("__lf_instance->${port.name}[__lf_port_idx]")}
                            |  }
                            |}
                        """.trimMargin()
                    } else {
                        """
                            |for (auto& __lf_instance : ${container.name}) {
                            |  for (auto& __lf_port : __lf_instance->${port.name}) {
                        ${" |    "..generateCode("__lf_port")}
                            |  }
                            |}
                        """.trimMargin()
                    }
                } else {
                    // is mulitport, but not in a bank
                    """
                        |for (auto& __lf_port : ${varRef.name}) {
                    ${" |  "..generateCode("__lf_port")}
                        |}
                    """.trimMargin()
                }
            } else {
                if (container?.isBank == true) {
                    // is in a bank, but not a multiport
                    """
                        |for (auto& __lf_instance : ${container.name}) {
                    ${" |  "..generateCode("__lf_instance->${port.name}")}
                        |}
                    """.trimMargin()
                } else {
                    // is just a normal port
                    generateCode(varRef.name)
                }
            }
        }
    }

    private fun declareTrigger(reaction: Reaction, trigger: TriggerRef): String =
        if (trigger is VarRef && trigger.variable is Port) {
            // if the trigger is a port, then it could be a multiport or contained in a bank
            iterateOverAllPortsAndApply(trigger) { port: String -> "${reaction.name}.declare_trigger(&$port);" }
        } else {
            // treat as single trigger otherwise
            "${reaction.name}.declare_trigger(&${trigger.name});"
        }

    private fun declareDependency(reaction: Reaction, dependency: VarRef): String {
        assert(dependency.variable is Port)
        // if the trigger is a port, then it could be a multiport or contained in a bank
        return iterateOverAllPortsAndApply(dependency) { port: String -> "${reaction.name}.declare_dependency(&$port);" }
    }

    private fun declareAntidependency(reaction: Reaction, antidependency: VarRef): String {
        val variable = antidependency.variable
        return if (variable is Port) {
            // if the trigger is a port, then it could be a multiport or contained in a bank
            iterateOverAllPortsAndApply(antidependency) { port: String -> "${reaction.name}.declare_antidependency(&$port);" }
        } else {
            // treat as single antidependency otherwise
            if (variable is Action) "${reaction.name}.declare_schedulable_action(&${antidependency.name});"
            else "${reaction.name}.declare_antidependency(&${antidependency.name});"
        }
    }

    private fun setDeadline(reaction: Reaction): String {
        val delay = reaction.deadline.delay
        val value = if (delay is ParameterReference) "__lf_inner.${delay.parameter.name}" else delay.toCppTime()
        return "${reaction.name}.set_deadline($value, [this]() { ${reaction.name}_deadline_handler(); });"
    }

    private fun assembleReaction(reaction: Reaction): String {
        val sources = reaction.sources.filter { it.variable is Port }
        return with(PrependOperator) {
            """
            |// ${reaction.name}
        ${" |"..reaction.triggers.joinToString(separator = "\n") { declareTrigger(reaction, it) }}
        ${" |"..sources.joinToString(separator = "\n") { declareDependency(reaction, it) }}
        ${" |"..reaction.effects.joinToString(separator = "\n") { declareAntidependency(reaction, it) }}
        ${" |"..if (reaction.deadline != null) setDeadline(reaction) else ""}
        """.trimMargin()
        }
    }

    private fun declareConnection(c: Connection, idx: Int): String =
        if (c.hasMultipleConnections) {
            declareMultiportConnection(c, idx)
        } else {
            val leftPort = c.leftPorts[0]
            val rightPort = c.rightPorts[0]

            if (c.requiresConnectionClass)
                """
                    // connection $idx
                    ${c.name}.bind_upstream_port(&${leftPort.name});
                    ${c.name}.bind_downstream_port(&${rightPort.name});
                """.trimIndent()
            else
                """
                    // connection $idx
                    ${leftPort.name}.bind_to(&${rightPort.name});
                """.trimIndent()
        }

    /**
     * Return the C++ type of a port.
     *
     * We cannot easily infer this type directly, because it might be used within a generic reactor. Instead of implementing
     * complex logic for finding the actual type, we return a decltype statement and let the C++ compiler do the job.
     */
    private val VarRef.portType: String
        get() = "reactor::Port<${dataType}>*"

    private fun declareMultiportConnection(c: Connection, idx: Int): String {
        // It should be safe to assume that all ports have the same type. Thus we just pick the
        // first left port to determine the type of the entire connection
        val portType = c.leftPorts[0].portType

        val leftPort = c.leftPorts[0].variable as Port
        val dataType = leftPort.inferredType.cppType

        // Generate code which adds all left hand ports and all right hand ports to a vector each. If we are handling multiports
        // within a bank, then we normally iterate over all banks in an outer loop and over all ports in an inner loop. However,
        // if the connection is a cross connection, than we change the order on the right side and iterate over ports before banks.
        return with(PrependOperator) {
            if (!c.requiresConnectionClass) {
                """
                    |// connection $idx
                    |std::vector<$portType> __lf_left_ports_$idx;
                ${" |"..c.leftPorts.joinWithLn { addAllPortsToVector(it, "__lf_left_ports_$idx") }}
                    |std::vector<$portType> __lf_right_ports_$idx;
                ${" |"..c.rightPorts.joinWithLn { addAllPortsToVector(it, "__lf_right_ports_$idx") }}
                    |lfutil::bind_multiple_ports(__lf_left_ports_$idx, __lf_right_ports_$idx, ${c.isIterated});
                """.trimMargin()
            } else {
                """
                    |// connection $idx
                    |std::vector<$portType> __lf_left_ports_$idx;
                ${" |"..c.leftPorts.joinWithLn { addAllPortsToVector(it, "__lf_left_ports_$idx") }}
                    |${c.name}.reserve(__lf_left_ports_$idx.size());
                    |for(size_t __lf_idx{0}; __lf_idx < __lf_left_ports_$idx.size(); __lf_idx++) {
                    |  ${c.name}.emplace_back("${c.name}" + std::to_string(__lf_idx), this, ${c.delay.toCppTime()});
                    |  ${c.name}.back().bind_upstream_port(__lf_left_ports_$idx[__lf_idx]);
                    |}
                    |std::vector<reactor::Connection<$dataType>*> __lf_connection_pointers_$idx;
                    |for (auto& connection : ${c.name}) {
                    |  __lf_connection_pointers_$idx.push_back(&connection);
                    |}
                    |std::vector<$portType> __lf_right_ports_$idx;
                ${" |"..c.rightPorts.joinWithLn { addAllPortsToVector(it, "__lf_right_ports_$idx") }}
                    |lfutil::bind_multiple_connections_with_ports(__lf_connection_pointers_$idx, __lf_right_ports_$idx, ${c.isIterated});
                """.trimMargin()
            }
        }
    }

    private fun addAllPortsToVector(varRef: VarRef, vectorName: String): String =
        iterateOverAllPortsAndApply(varRef) { port: String -> "${vectorName}.push_back(&$port);" }

    /**
     * Generate the definition of the reactor's assemble() method
     *
     * The body of this method will declare all triggers, dependencies and antidependencies to the runtime.
     */
    fun generateDefinition() = with(PrependOperator) {
        val indexedConnections = reactor.connections.withIndex()
        """
            |${reactor.templateLine}
            |void ${reactor.templateName}::assemble() {
        ${" |  "..reactor.reactions.joinToString("\n\n") { assembleReaction(it) }}
        ${" |  "..indexedConnections.joinToString("\n", prefix = "// connections\n") { declareConnection(it.value, it.index) }}
            |}
        """.trimMargin()
    }
}
