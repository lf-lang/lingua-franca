/*
 * Copyright (c) 2021, TU Dresden.
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

package org.lflang.generator.rust

import org.lflang.generator.UnsupportedGeneratorFeatureException
import org.lflang.getWidth
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.*
import kotlin.math.ceil

/**
 * Specialized thing to render port connections. This was
 * copy-pasted from the C++ generator,
 */
object PortEmitter {

    /*
     * todo refactor what can be reused
     *  Maybe with an abstract class, the only changed behavior
     *  between Rust & C++ for now is the syntax of the method
     *  calls & such.
     *
     * fixme some pieces of code that this generates without error are actually unsupported by the runtime
     */

    fun declareConnection(c: Connection): String {
        val lhsPorts = enumerateAllPortsFromReferences(c.leftPorts)
        val rhsPorts = enumerateAllPortsFromReferences(c.rightPorts)

        // If the connection is a broadcast connection, then repeat the lhs ports until it is equal
        // or greater to the number of rhs ports. Otherwise, continue with the unmodified list of lhs
        // ports
        val iteratedLhsPorts = if (c.isIterated) {
            val numIterations = ceil(rhsPorts.size.toDouble() / lhsPorts.size.toDouble()).toInt()
            (1..numIterations).flatMap { lhsPorts }
        } else {
            lhsPorts
        }

        // bind each pair of lhs and rhs ports individually
        return (iteratedLhsPorts zip rhsPorts).joinToString("\n") {
            "__assembler.bind_ports(&mut ${it.first.toCode()}, &mut ${it.second.toCode()})?;"
        }
    }

    fun declarePortRef(ref: ChildPortReference): String =
        with(ref) {
            val self = "&mut __self.$rustFieldName"
            val child = "&mut $rustChildName.$rustFieldOnChildName"

            if (isInput) "__assembler.bind_ports($self, $child)?;"
            else "__assembler.bind_ports($child, $self)?;"
        }

    /**
     * Get a list of PortReferences for the given list of variables
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
                if (container?.isBank == true) (0 until container.widthSpec.getValidWidth())
                else listOf<Int?>(null)
            val portIndices =
                if (port.isMultiport) (0 until port.widthSpec.getValidWidth())
                else listOf<Int?>(null)
            // calculate the Cartesian product af both index lists defined above
            // TODO iterate over banks or ports first?
            val indexPairs = portIndices.flatMap { portIdx -> bankIndexes.map { bankIdx -> portIdx to bankIdx } }
            ports.addAll(indexPairs.map { PortReference(port, it.first, container, it.second) })
        }
        return ports
    }

    /**
     * A data class for holding all information that is relevant for reverencing one specific port
     *
     * The port could be a member of a bank instance and it could be an instance of a multiport.
     * Thus, the information in this class includes a bank and port index. If the bank (or port)
     * index is null, then the referenced port is not part of a bank (or multiport).
     */
    private data class PortReference(val port: Port, val portIndex: Int?, val container: Instantiation?, val containerIndex: Int?)

    private fun PortReference.toCode(): String {
        val port = PortData.from(port)
        val portRef = if (port.isMultiport) "${port.rustFieldName}[$portIndex]" else port.rustFieldName
        return if (container != null) {
            val containerRef = if (container.isBank) "${container.name.escapeRustIdent()}[$containerIndex]" else container.name.escapeRustIdent()
            "$containerRef.$portRef"
        } else {
            "__self.$portRef"
        }
    }

    /**
     * Calculate the width of a multiport.
     * This reports an error on the receiving port if the width is not given as a literal integer.
     */
    private fun WidthSpec.getValidWidth(): Int =
        getWidth().takeIf { it >= 0 }
            ?: throw UnsupportedGeneratorFeatureException("Non-literal multiport widths")

}
