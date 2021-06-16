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
import org.lflang.label
import org.lflang.lf.*
import org.lflang.priority
import org.lflang.toText

/** A C++ code generator for reactions and their function bodies */
class CppReactionGenerator(
    private val reactor: Reactor,
    private val actionGenerator: CppActionGenerator
) {

    private val reactionsWithDeadlines = reactor.reactions.filter { it.deadline != null }

    private val Reaction.allTriggers get() = triggers.filterNot { it in effects }
    private val Reaction.allEffects get() = effects
    private val Reaction.allSources get() = effects.filterNot { it in effects }

    private val VarRef.cppType
        get() =
            when (val variable = this.variable) {
                is Timer  -> "reactor::Timer"
                is Action -> with(actionGenerator) { variable.cppType }
                else      -> TODO("Unexpected variable type")
            }

    private val TriggerRef.cppType
        get() = when {
            this.isStartup  -> "reactor::StartupAction"
            this.isShutdown -> "reactor::StartupAction"
            this is VarRef  -> cppType
            else            -> TODO("Unexpected variable type")
        }

    fun Reaction.getBodyParameters(): List<String> =
        allTriggers.map { "const ${it.cppType}& ${it.name}" } +
                allSources.map { "const ${it.cppType}& ${it.name}" } +
                allEffects.map { "${it.cppType}& ${it.name}" }

    private fun generateDeclaration(r: Reaction): String {
        val parameters = r.allTriggers.map{ it.name } + r.allSources.map{ it.name } + r.allEffects.map{ it.name }
        return """
            reactor::Reaction ${r.name}{"${r.label}", ${r.priority}, this, [this]() { 
              __lf_inner.${r.name}_body(${parameters.joinToString(", ")});
            }};
        """.trimIndent()
    }

    private fun generateBodyDeclaration(reaction: Reaction): String {
        val params = reaction.getBodyParameters()
        return when (params.size) {
            0    -> "void ${reaction.name}_body();"
            1    -> "void ${reaction.name}_body(${params[0]});"
            else -> with(PrependOperator) {
                """
                    |void ${reaction.name}_body(
                ${" |  "..params.joinToString(",\n")}); 
                """.trimMargin()
            }
        }
    }

    private fun generateBodyDefinition(reaction: Reaction): String {
        // TODO this doesn't work for contained ports
        // TODO this doesn't work for banks
        val params = reaction.getBodyParameters()
        val definitionSignature = when (params.size) {
            0    -> "void ${reactor.templateName}::Inner::${reaction.name}_body()"
            1    -> "void ${reactor.templateName}::Inner::${reaction.name}_body(${params[0]})"
            else -> with(PrependOperator) {
                """
                    |void ${reactor.templateName}::Inner::${reaction.name}_body(
                ${" |  "..params.joinToString(",\n")}) 
                """.trimMargin()
            }
        }

        return with(PrependOperator) {
            """
                |// reaction ${reaction.label}
                |${reactor.templateLine}
                |$definitionSignature {
            ${" |  "..reaction.code.toText()}
                |}
                |
            """.trimMargin()
        }
    }

    private fun generateDeadlineHandlerDefinition(reaction: Reaction): String = with(PrependOperator) {
        // TODO Should provide the same context as in reactions
        return """
            |${reactor.templateLine}
            |void ${reactor.templateName}::${reaction.name}_deadline_handler() {
        ${" |  "..reaction.deadline.code.toText()}
            |}
            |
        """.trimMargin()
    }

    /** Get all reaction declarations. */
    fun generateDeclarations() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reactions\n", postfix = "\n") { generateDeclaration(it) }

    /** Get all declarations of reaction bodies. */
    fun generateBodyDeclarations() =
        reactor.reactions.joinToString("\n", "// reaction bodies\n", "\n") { generateBodyDeclaration(it) }

    /** Get all definitions of reaction bodies. */
    fun generateBodyDefinitions() =
        reactor.reactions.joinToString(separator = "\n", postfix = "\n") { generateBodyDefinition(it) }

    /** Get all declarations of deadline handlers. */
    fun generateDeadlineHandlerDeclarations(): String =
        reactionsWithDeadlines.joinToString("\n", "// deadline handlers\n", "\n") {
            "void ${it.name}_deadline_handler();"
        }

    /** Get all definitions of deadline handlers. */
    fun generateDeadlineHandlerDefinitions() =
        reactionsWithDeadlines.joinToString(separator = "\n", postfix = "\n") { generateDeadlineHandlerDefinition(it) }
}