/*************
 * Copyright (c) 2019-2021, TU Dresden.

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

import org.lflang.inferredType
import org.lflang.isOfTimeType
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor

/** A C++ code generator for reactor parameters */
class CppParameterGenerator(private val reactor: Reactor) {

    companion object {

        /**
         * Create a list of initializers for the given parameter
         *
         * TODO This is redundant to ExpressionGenerator.getInitializerList
         */
        private fun Parameter.getInitializerList() = init.map {
            if (isOfTimeType) it.toTime()
            else it.toCppCode()
        }

        /** Type of the parameter in C++ code */
        val Parameter.targetType get(): String = this.inferredType.cppType

        /** Get the default value of the receiver parameter in C++ code */
        val Parameter.defaultValue: String
            get() =
                if (braces?.size == 2) "$targetType{${getInitializerList().joinToString(", ")}}"
                else "$targetType(${getInitializerList().joinToString(", ")})"

        /** Get a C++ type that is a const reference to the parameter type */
        val Parameter.constRefType: String
            get() =
                "typename std::add_lvalue_reference<typename std::add_const<$targetType>::type>::type"
    }

    /** Generate all parameter declarations */
    fun generateDeclarations() =
        reactor.parameters.joinToString("\n", "// parameters\n", "\n") {
            "typename std::add_const<${it.targetType}>::type ${it.name};"
        }

    /** Generate all constructor initializers for parameters */
    fun generateInitializers() =
        reactor.parameters.joinToString("\n", "// parameters\n", "\n") {
            ", ${it.name}(${it.name})"
        }
}
