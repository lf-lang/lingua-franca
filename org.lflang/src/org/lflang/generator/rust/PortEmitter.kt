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

import org.lflang.generator.TargetCode
import org.lflang.generator.locationInfo
import org.lflang.headAndTail
import org.lflang.isBank
import org.lflang.lf.Connection
import org.lflang.lf.Instantiation
import org.lflang.lf.Port
import org.lflang.lf.VarRef

/**
 * Specialized thing to render port connections.
 */
object PortEmitter : RustEmitterBase() {

    /**
     * Returns the Rust code that declares the given connection.
     */
    fun Connection.declareConnection(assembler: String): String {
        val lhsPorts = leftPorts.iterAllPorts()
        val rhsPorts = rightPorts.iterAllPorts()

        val methodName =
            if (isIterated) "bind_ports_iterated"
            else "bind_ports_zip"

        return """
                |{ ${locationInfo().lfTextComment()}
                |    let up = $lhsPorts;
                |    let down = $rhsPorts;
                |    $assembler.$methodName(up, down)?;
                |}
            """.trimMargin()
    }

    fun ChildPortReference.declarePortRef(assembler: String): String {
        val self = "&mut __self.$rustFieldName"
        val child = "&mut $rustChildName.$rustFieldOnChildName"

        return if (isGeneratedAsMultiport) {
            var lhsPorts = "$child.iter_mut()"
            var rhsPorts = "$self.iter_mut()"

            if (isContainedInBank && !isMultiport) {
                lhsPorts = "unsafe_iter_bank!($rustChildName # $rustFieldOnChildName)"
            } else if (isContainedInBank && isMultiport) {
                lhsPorts = "unsafe_iter_bank!($rustChildName # ($rustFieldOnChildName)+)"
            }

            if (isInput) {
                lhsPorts = rhsPorts.also { rhsPorts = lhsPorts }
            }

            "$assembler.bind_ports_zip($lhsPorts, $rhsPorts)?;"
        } else {
            if (isInput) "$assembler.bind_ports($self, $child)?;"
            else "$assembler.bind_ports($child, $self)?;"
        }
    }

    /**
     * Produce a Rust expression that creates a single iterator
     * by chaining the result of [iterPorts] for all the components
     * of [this] list.
     */
    private fun List<VarRef>.iterAllPorts(): TargetCode {
        if (size == 1) return this[0].iterPorts()

        val (hd, tl) = headAndTail()

        return tl.fold(hd.iterPorts()) { acc, varRef ->
            "$acc.chain(${varRef.iterPorts()})"
        }
    }

    /**
     * Produce an expression that creates an `Iterator<Item=&mut Port<T>>`,
     * which iterates over all the individual channels of the
     * port reference. This expands port banks and multiports
     * into individual channels.
     */
    private fun VarRef.iterPorts(): TargetCode {
        val container: Instantiation? = container
        val port = PortData.from(variable as Port)

        if (container?.isBank == true && port.isGeneratedAsMultiport && isInterleaved) {
            return "unsafe_iter_bank!(${container.name} # interleaved(${port.rustFieldName}))"
        } else if (container?.isBank == true && port.isGeneratedAsMultiport) {
            return "unsafe_iter_bank!(${container.name} # (${port.rustFieldName})+)"
        } else if (container?.isBank == true) {
            return "unsafe_iter_bank!(${container.name} # ${port.rustFieldName})"
        }

        // todo this is missing some tests where we try to borrow several multiports from same reactor
        val ref = (container?.name ?: "__self") + "." + port.rustFieldName

        return if (port.isGeneratedAsMultiport) {
            "$ref.iter_mut()"
        } else {
            "std::iter::once(&mut $ref)"
        }
    }
}
