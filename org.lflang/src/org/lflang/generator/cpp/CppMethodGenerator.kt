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

import org.lflang.InferredType
import org.lflang.generator.PrependOperator
import org.lflang.lf.Method
import org.lflang.lf.MethodArgument
import org.lflang.lf.Reactor
import org.lflang.toText

/** A C++ code generator for state variables */
class CppMethodGenerator(private val reactor: Reactor) {

    private val Method.targetType: String get() = if (`return` != null) InferredType.fromAST(`return`).targetType else "void"
    private val MethodArgument.targetType: String get() = InferredType.fromAST(type).targetType

    private val Method.cppArgs get() = this.arguments.map { "${it.targetType} ${it.name}" }
    private val Method.constQualifier get() = if (isConst) " const" else ""

    private fun generateDefinition(method: Method): String = with(PrependOperator) {
        with(method) {
            """
                |${reactor.templateLine}
                |$targetType ${reactor.templateName}::Inner::$name(${cppArgs.joinToString(", ")})$constQualifier {
            ${" |  "..code.toText()}
                |}
            """.trimMargin()
        }
    }

    private fun generateDeclaration(method: Method): String = with(method) {
        "$targetType $name(${cppArgs.joinToString(", ")})$constQualifier;"
    }

    /** Get all method definitions */
    fun generateDefinitions() =
        reactor.methods.joinToString("\n", "// methods\n", "\n") { generateDefinition(it) }

    /** Get all method declarations */
    fun generateDeclarations() =
        reactor.methods.joinToString("\n", "// methods\n", "\n") { generateDeclaration(it) }
}