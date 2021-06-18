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
    private val portGenerator: CppPortGenerator
) {

    private val reactionsWithDeadlines = reactor.reactions.filter { it.deadline != null }


    private val VarRef.isContainedRef: Boolean get() = container != null
    private val TriggerRef.isContainedRef: Boolean get() = this is VarRef && isContainedRef

    private val Reaction.allUncontainedTriggers get() = triggers.filterNot { it in effects || it.isContainedRef }
    private val Reaction.allUncontainedEffects get() = effects.filterNot { it.isContainedRef }
    private val Reaction.allUncontainedSources get() = sources.filterNot { it in effects || it.isContainedRef }
    private val Reaction.allVariableReferences get() = (effects + sources + triggers.mapNotNull { it as? VarRef }).distinct()
    private val Reaction.allReferencedContainers get() = allVariableReferences.mapNotNull { it.container }.distinct()

    private fun Reaction.getAllReferencedVariablesForContainer(container: Instantiation) =
        allVariableReferences.filter { it.container == container }.distinct()

    private fun Instantiation.isReadOnly(r: Reaction) = r.allReferencedContainers.any { it == this }

    private fun Reaction.getViewName(container: Instantiation) = "View_of_${name}_on_${container.name}"

    private val VarRef.cppType
        get() =
            when (val variable = this.variable) {
                is Timer  -> "reactor::Timer"
                is Action -> with(CppActionGenerator) { variable.cppType }
                is Port   -> with(portGenerator) { variable.cppType }
                else      -> AssertionError("Unexpected variable type")
            }

    private val TriggerRef.cppType
        get() = when {
            this.isStartup  -> "reactor::StartupAction"
            this.isShutdown -> "reactor::ShutdownAction"
            this is VarRef  -> cppType
            else            -> AssertionError("Unexpected trigger type")
        }

    private fun Reaction.getBodyParameters(): List<String> =
        allUncontainedTriggers.map { "const ${it.cppType}& ${it.name}" } +
                allUncontainedSources.map { "const ${it.cppType}& ${it.name}" } +
                allUncontainedEffects.map { "${it.cppType}& ${it.name}" } +
                allReferencedContainers.map {
                    if (it.isReadOnly(this))
                        "const ${getViewName(it)}& ${it.name}"
                    else
                        "${getViewName(it)}& ${it.name}"
                }

    private fun generateDeclaration(r: Reaction): String {
        val parameters = r.allUncontainedTriggers.map { it.name } +
                r.allUncontainedSources.map { it.name } +
                r.allUncontainedEffects.map { it.name } +
                r.allReferencedContainers.map {
                    if (it.isReadOnly(r))
                        "reinterpret_cast<const ${r.getViewName(it)}&>(*${it.name})"
                    else
                        "reinterpret_cast<${r.getViewName(it)}&>(*${it.name})"
                }
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

    private fun getFunctionSignature(reaction: Reaction, postfix: String): String {
        val params = reaction.getBodyParameters()
        return when (params.size) {
            0    -> "void ${reactor.templateName}::Inner::${reaction.name}_$postfix()"
            1    -> "void ${reactor.templateName}::Inner::${reaction.name}_$postfix(${params[0]})"
            else -> with(PrependOperator) {
                """
                    |void ${reactor.templateName}::Inner::${reaction.name}_$postfix(
                ${" |  "..params.joinToString(",\n")}) 
                """.trimMargin()
            }
        }
    }

    private fun generateBodyDefinition(reaction: Reaction): String {
        // TODO this doesn't work for banks
        return with(PrependOperator) {
            """
                |// reaction ${reaction.label}
                |${reactor.templateLine}
            ${" |"..getFunctionSignature(reaction, "body")} {
            ${" |  "..reaction.code.toText()}
                |}
                |
            """.trimMargin()
        }
    }

    private fun generateDeadlineHandlerDefinition(reaction: Reaction): String = with(PrependOperator) {
        // TODO this doesn't work for banks
        return """
            |${reactor.templateLine}
        ${" |"..getFunctionSignature(reaction, "deadline_handler")} {
        ${" |  "..reaction.deadline.code.toText()}
            |}
            |
        """.trimMargin()
    }

    private fun generateViewForContainer(r: Reaction, container: Instantiation): String {
        val reactorClass = container.reactorClass.name
        val variables = r.getAllReferencedVariablesForContainer(container)
        return with(PrependOperator) {
            """
                |struct ${r.getViewName(container)} : protected $reactorClass {
            ${" |  "..variables.joinToString("\n") { "using $reactorClass::${it.variable.name};" }}
                |};
            """.trimMargin()
        }
    }

    private fun generateViews(r: Reaction) =
        r.allReferencedContainers.joinToString("\n") { generateViewForContainer(r, it) }

    fun generateReactionViews() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") { generateViews(it) }

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