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
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor

/** A code genarator for reactor instances */
class CppInstanceGenerator(
    private val reactor: Reactor,
    private val fileConfig: CppFileConfig,
    private val errorReporter: ErrorReporter,
) {

    val Instantiation.cppType: String
        get() {
            return if (reactor.isGeneric)
                """${reactor.name}<${typeParms.joinToString(", ") { it.toText() }}>"""
            else
                reactor.name
        }

    private fun generateDeclaration(inst: Instantiation): String {
        return if (inst.isBank)
            "std::array<std::unique_ptr<${inst.cppType}>, ${inst.getValidWidth()}> ${inst.name};"
        else
            "std::unique_ptr<${inst.cppType}> ${inst.name};"
    }

    private fun Instantiation.getParameterValue(param: Parameter, instanceId: Int? = null): String {
        val assignment = this.parameters.firstOrNull { it.lhs === param }

        return if (instanceId != null && param.name == "instance") {
            // If we are in a bank instantiation (instanceId != null), then assign the instanceId
            // to the parameter named "instance"
            instanceId.toString()
        } else if (assignment == null) {
            // If no assignment was found, then the parameter is not overwritten and we assign the
            // default value
            with(CppParameterGenerator) { param.defaultValue }
        } else {
            // Otherwise, we use the assigned value.
            val initializers = assignment.rhs.map { if (param.isOfTimeType) it.toTime() else it.toCode() }
            with(CppParameterGenerator) { param.generateInstance(initializers) }
        }
    }

    private fun generateInitializer(inst: Instantiation): String {
        val parameters = inst.reactor.parameters
        return if (inst.isBank) {
            val initializations = if (parameters.isEmpty()) {
                (0 until inst.getValidWidth()).joinToString(", ") {
                    """std::make_unique<${inst.cppType}>("${inst.name}_$it", this)"""
                }
            } else {
                (0 until inst.getValidWidth()).joinToString(", ") {
                    val params = parameters.joinToString(", ") { param -> inst.getParameterValue(param, it) }
                    """std::make_unique<${inst.cppType}>("${inst.name}", this, $params)"""
                }
            }
            """, ${inst.name}{{$initializations}}"""
        } else {
            if (parameters.isEmpty())
                """, ${inst.name}(std::make_unique<${inst.cppType}>("${inst.name}", this))"""
            else {
                val params = parameters.joinToString(", ") { inst.getParameterValue(it) }
                """, ${inst.name}(std::make_unique<${inst.cppType}>("${inst.name}", this, $params))"""
            }
        }
    }

    /**
     * Calculate the width of a multiport.
     *
     * This reports an error on the receiving port if the width is not given as a literal integer.
     */
    fun Instantiation.getValidWidth(): Int {
        val width = widthSpec.getWidth()
        if (width < 0) {
            // TODO Support parameterized widths
            errorReporter.reportError(
                this,
                "The C++ target only supports bank widths specified as literal integer values for now"
            )
        }
        return width
    }

    /** Generate C++ include statements for each reactor that is instantiated */
    fun generateIncludes(): String =
        reactor.instantiations.map { fileConfig.getReactorHeaderPath(it.reactor) }
            .distinct()
            .joinToString(separator = "\n") { """#include "${it.toUnixString()}" """ }

    /** Generate declaration statements for all reactor instantiations */
    fun generateDeclarations(): String {
        return reactor.instantiations.joinToString(
            prefix = "// reactor instances\n",
            separator = "\n"
        ) { generateDeclaration(it) }
    }

    /** Generate constructor initializers for all reactor instantiations */
    fun generateInitializers(): String =
        reactor.instantiations.joinToString(prefix = "//reactor instances\n", separator = "\n") { generateInitializer(it) }
}
