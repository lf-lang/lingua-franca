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
import org.lflang.lf.Reactor
import java.lang.RuntimeException

/** A code generator for reactor instances */
class CppInstanceGenerator(
    private val reactor: Reactor,
    private val fileConfig: CppFileConfig,
    private val errorReporter: ErrorReporter
) {
    private val Instantiation.isEnclave: Boolean get() = AttributeUtils.isEnclave(this)

    val Instantiation.cppType: String
        get() {
            return if (reactor.isGeneric)
                """${reactor.name}<${typeParms.joinToString(", ") { it.toText() }}>"""
            else
                reactor.name
        }

    private fun generateDeclaration(inst: Instantiation): String = with(inst) {
        val instance = if (isBank) "std::vector<std::unique_ptr<$cppType>> $name;" else "std::unique_ptr<$cppType> $name;"
        if (isEnclave) {
            return """
                reactor::Environment __lf_env_$name;
                $instance
            """.trimIndent()
        }
        return instance
    }

    private fun Instantiation.getParameterStruct(): String {
        val assignments = parameters.mapNotNull {
            when {
                it.rhs.isParens || it.rhs.isBraces -> {
                    errorReporter.reportError(it, "Parenthesis based initialization is not allowed here!")
                    null
                }

                it.rhs.exprs.size != 1             -> {
                    errorReporter.reportError(it, "Expected exactly one expression.")
                    null
                }

                else                               -> Pair(
                    it.lhs.name,
                    CppTypes.getTargetExpr(it.rhs.exprs[0], it.lhs.inferredType)
                )
            }
        }.toMap().toMutableMap()

        // If this is a bank instantiation and the instantiated reactor defines a "bank_index" parameter, we have to set
        // bank_index here explicitly.
        if (isBank && reactor.hasBankIndexParameter())
            assignments["bank_index"] = "__lf_idx"

        // by iterating over the reactor parameters we make sure that the parameters are assigned in declaration order
        return reactor.parameters.mapNotNull {
            if (it.name in assignments) ".${it.name} = ${assignments[it.name]}" else null
        }.joinToString(", ", "$cppType::Parameters{", "}")
    }

    private fun generateInitializer(inst: Instantiation): String? = with(inst) {
        when {
            !isBank && !isEnclave -> """, $name(std::make_unique<$cppType>("$name}", this, ${getParameterStruct()}))"""
            !isBank && isEnclave  -> """
                    , __lf_env_$name(this->fqn() + ".$name", this->environment())
                    , $name(std::make_unique<$cppType>("$name}", &__lf_env_$name, ${getParameterStruct()}))
                """.trimIndent()

            isBank && isEnclave   -> """, __lf_env_$name(this->fqn() + ".$name", this->environment())"""
            else                  -> null
        }
    }

    private fun generateConstructorInitializer(inst: Instantiation): String {
        with(inst) {
            assert(isBank)
            val containerRef = if (isEnclave) "&__lf_env_$name" else "this"
            val emplaceLine =
                "$name.emplace_back(std::make_unique<$cppType>(__lf_inst_name, $containerRef, ${inst.getParameterStruct()}));"

            val width = inst.widthSpec.toCppCode()
            return """
                // initialize instance $name
                $name.reserve($width);
                for (size_t __lf_idx = 0; __lf_idx < $width; __lf_idx++) {
                  std::string __lf_inst_name = "${name}_" + std::to_string(__lf_idx);
                  $emplaceLine
                }
            """.trimIndent()
        }
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

    fun generateConstructorInitializers() =
        reactor.instantiations.filter { it.isBank }.joinWithLn { generateConstructorInitializer(it) }

    /** Generate constructor initializers for all reactor instantiations */
    fun generateInitializers(): String =
        reactor.instantiations.mapNotNull { generateInitializer(it) }
            .joinToString(prefix = "//reactor instances\n", separator = "\n")
}
