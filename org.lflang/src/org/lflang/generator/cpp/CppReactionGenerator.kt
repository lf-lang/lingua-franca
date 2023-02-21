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
import org.lflang.joinWithLn
import org.lflang.label
import org.lflang.lf.Action
import org.lflang.lf.BuiltinTrigger
import org.lflang.lf.BuiltinTriggerRef
import org.lflang.lf.Instantiation
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.Timer
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef
import org.lflang.priority
import org.lflang.toText

/** A C++ code generator for reactions and their function bodies */
class CppReactionGenerator(
    private val reactor: Reactor,
    private val portGenerator: CppPortGenerator,
    private val instanceGenerator: CppInstanceGenerator
) {

    private val reactionsWithDeadlines = reactor.reactions.filter { it.deadline != null }

    private val VarRef.isContainedRef: Boolean get() = container != null
    private val TriggerRef.isContainedRef: Boolean get() = this is VarRef && isContainedRef

    private fun VarRef.isEffectOf(reaction: Reaction): Boolean =
        reaction.effects.any { name == it.name && container?.name == it.container?.name }

    private fun TriggerRef.isEffectOf(reaction: Reaction): Boolean = this is VarRef && isEffectOf(reaction)

    private val Reaction.allUncontainedTriggers get() = triggers.filterNot { it.isEffectOf(this) || it.isContainedRef }
    private val Reaction.allUncontainedEffects get() = effects.filterNot { it.isContainedRef }
    private val Reaction.allUncontainedSources get() = sources.filterNot { it.isEffectOf(this) || it.isContainedRef }
    private val Reaction.allVariableReferences get() = (effects + sources + triggers.mapNotNull { it as? VarRef }).distinct()
    private val Reaction.allReferencedContainers get() = allVariableReferences.mapNotNull { it.container }.distinct()

    private fun Reaction.getAllReferencedVariablesForContainer(container: Instantiation) =
        allVariableReferences.filter { it.container == container }.distinct()

    private fun Reaction.getViewClassName(container: Instantiation) = "__lf_view_of_${name}_on_${container.name}_t"
    private fun Reaction.getViewInstanceName(container: Instantiation) = "__lf_view_of_${name}_on_${container.name}"

    private val VarRef.cppType
        get() =
            when (val variable = this.variable) {
                is Timer  -> "reactor::Timer"
                is Action -> with(CppActionGenerator) { variable.cppType }
                is Port   -> with(portGenerator) { variable.cppInterfaceType }
                else      -> AssertionError("Unexpected variable type")
            }

    private val TriggerRef.cppType
        get() = when {
            this is BuiltinTriggerRef && this.type == BuiltinTrigger.STARTUP  -> "reactor::StartupTrigger"
            this is BuiltinTriggerRef && this.type == BuiltinTrigger.SHUTDOWN -> "reactor::ShutdownTrigger"
            this is VarRef                                                    -> cppType
            else                                                              -> AssertionError("Unexpected trigger type")
        }

    private fun Reaction.getBodyParameters(): List<String> =
        allUncontainedTriggers.map { "[[maybe_unused]] const ${it.cppType}& ${it.name}" } +
                allUncontainedSources.map { "const ${it.cppType}& ${it.name}" } +
                allUncontainedEffects.map { "${it.cppType}& ${it.name}" } +
                allReferencedContainers.map {
                    if (it.isBank) "const std::vector<${getViewClassName(it)}>& ${it.name}"
                    else "${getViewClassName(it)}& ${it.name}"
                }

    private fun generateDeclaration(r: Reaction): String {
        with(r) {
            val parameters = allUncontainedTriggers.map { it.name } +
                    allUncontainedSources.map { it.name } +
                    allUncontainedEffects.map { it.name } +
                    allReferencedContainers.map { getViewInstanceName(it) }
            val body = "void ${name}_body() { __lf_inner.${name}_body(${parameters.joinToString(", ")}); }"
            val deadlineHandler =
                "void ${name}_deadline_handler() { __lf_inner.${name}_deadline_handler(${parameters.joinToString(", ")}); }"

            return if (deadline == null)
                """
                    $body
                    reactor::Reaction $name{"$label", $priority, this, [this]() { ${name}_body(); }}; 
                """.trimIndent()
            else
                """
                    $body
                    $deadlineHandler
                    reactor::Reaction $name{"$label", $priority, this, [this]() { ${name}_body(); }}; 
                """.trimIndent()
        }
    }

    private fun generateFunctionDeclaration(reaction: Reaction, postfix: String): String {
        val params = reaction.getBodyParameters()
        return when (params.size) {
            0    -> "void ${reaction.name}_$postfix();"
            1    -> "void ${reaction.name}_$postfix(${params[0]});"
            else -> with(PrependOperator) {
                """
                    |void ${reaction.name}_$postfix(
                ${" |  "..params.joinToString(",\n")}); 
                """.trimMargin()
            }
        }
    }

    private fun getFunctionDefinitionSignature(reaction: Reaction, postfix: String): String {
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
        return with(PrependOperator) {
            """
                |// reaction ${reaction.label}
                |${reactor.templateLine}
            ${" |"..getFunctionDefinitionSignature(reaction, "body")} {
            ${" |  "..reaction.code.toText()}
                |}
                |
            """.trimMargin()
        }
    }

    private fun generateDeadlineHandlerDefinition(reaction: Reaction): String = with(PrependOperator) {
        return """
            |${reactor.templateLine}
        ${" |"..getFunctionDefinitionSignature(reaction, "deadline_handler")} {
        ${" |  "..reaction.deadline.code.toText()}
            |}
            |
        """.trimMargin()
    }

    private fun generateViewForContainer(r: Reaction, container: Instantiation): String {
        val reactorClass = with(instanceGenerator) { container.cppType }
        val viewClass = r.getViewClassName(container)
        val viewInstance = r.getViewInstanceName(container)

        val variables = r.getAllReferencedVariablesForContainer(container)
        val instantiations = variables.map {
            val type = "decltype($reactorClass::${it.variable.name})& ${it.variable.name};"
            if (it.isEffectOf(r)) type else "const $type"
        }
        val initializers = variables.map { "${it.variable.name}(reactor->${it.variable.name})" }

        val viewDeclaration =
            if (container.isBank) "std::vector<$viewClass> $viewInstance;"
            else "$viewClass $viewInstance;"

        return with(PrependOperator) {
            """
                |struct $viewClass {
            ${" |  "..instantiations.joinToString("\n")}
                |  $viewClass($reactorClass* reactor) :
            ${" |    "..initializers.joinToString(",\n")}
                |  {}
                |};
                |$viewDeclaration
            """.trimMargin()
        }
    }

    private fun generateViews(r: Reaction) =
        r.allReferencedContainers.joinWithLn { generateViewForContainer(r, it) }

    private fun generateViewInitializers(r: Reaction) =
        r.allReferencedContainers.filterNot { it.isBank }
            .joinWithLn { ", ${r.getViewInstanceName(it)}(${it.name}.get()) " }

    private fun generateViewConstructorInitializers(r: Reaction) =
        r.allReferencedContainers.filter { it.isBank }
            .joinWithLn {
                val viewInstance = r.getViewInstanceName(it)
                """
                    $viewInstance.reserve(${it.name}.size());
                    for (auto& __lf_instance : ${it.name}) {
                      $viewInstance.emplace_back(__lf_instance.get());
                    }
                """.trimIndent()
            }

    fun generateReactionViews() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") { generateViews(it) }

    fun generateReactionViewInitializers() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") {
            generateViewInitializers(it)
        }

    fun generateReactionViewForwardDeclarations(): String {
        val classNames = reactor.reactions.map { r -> r.allReferencedContainers.map { r.getViewClassName(it) } }.flatten()
        if (classNames.isEmpty()) {
            return ""
        }
        return classNames.joinWithLn(prefix = "// reaction view forward declarations\n") { "struct $it;" }
    }

    fun generateReactionViewConstructorInitializers() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") {
            generateViewConstructorInitializers(it)
        }

    /** Get all reaction declarations. */
    fun generateDeclarations() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reactions\n", postfix = "\n") { generateDeclaration(it) }

    /** Get all declarations of reaction bodies. */
    fun generateBodyDeclarations() =
        reactor.reactions.joinToString("\n", "// reaction bodies\n", "\n") {
            generateFunctionDeclaration(it, "body")
        }

    /** Get all definitions of reaction bodies. */
    fun generateBodyDefinitions() =
        reactor.reactions.joinToString(separator = "\n", postfix = "\n") { generateBodyDefinition(it) }

    /** Get all declarations of deadline handlers. */
    fun generateDeadlineHandlerDeclarations(): String =
        reactionsWithDeadlines.joinToString("\n", "// deadline handlers\n", "\n") {
            generateFunctionDeclaration(it, "deadline_handler")
        }

    /** Get all definitions of deadline handlers. */
    fun generateDeadlineHandlerDefinitions() =
        reactionsWithDeadlines.joinToString(separator = "\n", postfix = "\n") { generateDeadlineHandlerDefinition(it) }
}
