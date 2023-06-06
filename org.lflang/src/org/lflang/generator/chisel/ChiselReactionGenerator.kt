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

package org.lflang.generator.chisel

import org.lflang.*
import org.lflang.generator.cpp.name
import org.lflang.lf.*

/** A C++ code generator for reactions and their function bodies */
class ChiselReactionGenerator(
    private val reactor: Reactor,
//    private val portGenerator: CppPortGenerator,
//    private val instanceGenerator: CppInstanceGenerator
) {

    fun generateDeclarations(): String =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reactions\n", postfix = "\n") { generateDeclaration(it) }

    fun generatePrecedenceConstraints(): String =
        if (reactor.reactions.size > 1) {
            val builder = StringBuilder()
            for (r in reactor.reactions) {
                if (r == reactor.reactions.first())
                    builder.append(r.name)
                else
                    builder.append("> ${r.name}")
            }
            builder.toString()
        } else {
            ""
        }

    private fun generateReactionConfig(r: Reaction): String {
        val nPrecedenceInPorts = if (r.indexInContainer > 1) 1 else 0
        val nPrecedenceOutPorts = if (r.indexInContainer < r.containingReactor.reactions.size-1) 1 else 0
        return "ReactionConfig(nPrecedenceIn = $nPrecedenceInPorts, nPrecedenceOut = $nPrecedenceOutPorts)"
    }
    private fun generateClassDefinition(r: Reaction): String =
        "class ${r.getClassName}(c: ReactionConfig) extends Reaction(c)"

    private fun generateInputPortIO(p: Port): String =
        "val ${p.name} = new EventReadMaster(${p.getDataType}, ${p.getTokenType})"

    private fun generateOutputPortIO(p: Port): String =
        "val ${p.name} = new EventWriteMaster(${p.getDataType}, ${p.getTokenType})"

    private fun generateStateIO(s: StateVar): String =
        "require(false)"

    private fun generateTimerIO(t: Timer): String =
        "val ${t.name} = new EventReadMaster(${t.getDataType}, ${t.getTokenType})"

    private fun generateTriggerIOs(r: Reaction): String =
        r.triggers.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Triggers \n", postfix = "\n") { generateInputPortIO(it) }

    private fun generateSourceIOs(r: Reaction): String =
        r.sources.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Sources \n", postfix = "\n") { generateInputPortIO(it) }

    private fun generateEffectIOs(r: Reaction): String =
        r.effects.map{it as VarRef}.map{it.variable}.filterIsInstance<Port>().joinToString(separator = "\n", prefix = "// Effects \n", postfix = "\n") { generateOutputPortIO(it) }

    private fun generateTimerIOs(r: Reaction): String =
        r.triggers.map{it as VarRef}.map{it.variable}.filterIsInstance<Timer>().joinToString(separator = "\n", prefix = "// Timers \n", postfix = "\n") { generateTimerIO(it) }

    private fun generateStateIOs(r: Reaction): String =
        reactor.stateVars.joinToString(separator = "\n", prefix = "// State IO \n", postfix = "\n") { generateStateIO(it) }
    private fun generateReactionIOClass(r: Reaction): String =
        """
            | // IO definition
            | class ${r.getIOClassName} extends Bundle {
            | ${generateTriggerIOs(r)}
            | ${generateSourceIOs(r)}
            | ${generateEffectIOs(r)}
            | ${generateTimerIOs(r)}
            | ${generateStateIOs(r)}
            | }
            |
        """.trimMargin()

    private fun generateReactionIODeclaration(r: Reaction): String =
        "val io = IO(new ${r.getIOClassName}(c))\n"

    private fun generateIOInScope(r: Reaction): String =
        (r.triggers + r.sources + r.effects).joinToString(separator = "\n", prefix = "// Bring IO into scope \n", postfix = "\n")
        { "val ${it.name} = io.${it.name}" } +
        reactor.stateVars.joinToString(separator = "\n", postfix = "\n") {"val ${it.name} = io.${it.name}"}

    private fun generateInstance(r: Reaction): String =
        "val ${r.getInstanceName} = Module(new ${r.getClassName}(${generateReactionConfig(r)})"

    private fun generateReactionBody(reaction: Reaction): String =
        """
            | def reactionBody(): Unit {
            |   ${reaction.code.body}
            | }
        """.trimIndent()

    private fun generateDeclaration(r: Reaction): String =
        generateClassDefinition(r) +
        "\n{\n" +
        generateReactionIOClass(r) +
        generateReactionIODeclaration(r) +
        generateIOInScope(r) +
        generateReactionBody(r) +
        "\nreactionMain" +
        "\n}\n" +
        generateInstance(r)
}
//    private val reactionsWithDeadlines = reactor.reactions.filter { it.deadline != null }
//
//    private val VarRef.isContainedRef: Boolean get() = container != null
//    private val TriggerRef.isContainedRef: Boolean get() = this is VarRef && isContainedRef
//
//    private fun VarRef.isEffectOf(reaction: Reaction): Boolean =
//        reaction.effects.any { name == it.name && container?.name == it.container?.name }
//
//    private fun TriggerRef.isEffectOf(reaction: Reaction): Boolean = this is VarRef && isEffectOf(reaction)
//
//    private val Reaction.allUncontainedTriggers get() = triggers.filterNot { it.isEffectOf(this) || it.isContainedRef }
//    private val Reaction.allUncontainedEffects get() = effects.filterNot { it.isContainedRef }
//    private val Reaction.allUncontainedSources get() = sources.filterNot { it.isEffectOf(this) || it.isContainedRef }
//    private val Reaction.allVariableReferences get() = (effects + sources + triggers.mapNotNull { it as? VarRef }).distinct()
//    private val Reaction.allReferencedContainers get() = allVariableReferences.mapNotNull { it.container }.distinct()
//
//    private fun Reaction.getAllReferencedVariablesForContainer(container: Instantiation) =
//        allVariableReferences.filter { it.container == container }.distinct()
//
//    private fun Reaction.getViewClassName(container: Instantiation) = "__lf_view_of_${codeName}_on_${container.name}_t"
//    private fun Reaction.getViewInstanceName(container: Instantiation) = "__lf_view_of_${codeName}_on_${container.name}"
//
//    private val VarRef.cppType
//        get() =
//            when (val variable = this.variable) {
//                is Timer  -> "reactor::Timer"
//                is Action -> with(CppActionGenerator) { variable.cppType }
//                is Port   -> with(portGenerator) { variable.cppInterfaceType }
//                else      -> AssertionError("Unexpected variable type")
//            }
//
//    private val TriggerRef.cppType
//        get() = when {
//            this is BuiltinTriggerRef && this.type == BuiltinTrigger.STARTUP  -> "reactor::StartupTrigger"
//            this is BuiltinTriggerRef && this.type == BuiltinTrigger.SHUTDOWN -> "reactor::ShutdownTrigger"
//            this is VarRef                                                    -> cppType
//            else                                                              -> AssertionError("Unexpected trigger type")
//        }
//
//    private fun Reaction.getBodyParameters(): List<String> =
//        allUncontainedTriggers.map { "[[maybe_unused]] const ${it.cppType}& ${it.name}" } +
//                allUncontainedSources.map { "const ${it.cppType}& ${it.name}" } +
//                allUncontainedEffects.map { "${it.cppType}& ${it.name}" } +
//                allReferencedContainers.map {
//                    if (it.isBank) "const std::vector<${getViewClassName(it)}>& ${it.name}"
//                    else "${getViewClassName(it)}& ${it.name}"
//                }
//
//    private fun generateDeclaration(r: Reaction): String {
//        with(r) {
//            val parameters = allUncontainedTriggers.map { it.name } +
//                    allUncontainedSources.map { it.name } +
//                    allUncontainedEffects.map { it.name } +
//                    allReferencedContainers.map { getViewInstanceName(it) }
//            val body = "void ${codeName}_body() { __lf_inner.${codeName}_body(${parameters.joinToString(", ")}); }"
//            val deadlineHandler =
//                "void ${codeName}_deadline_handler() { __lf_inner.${codeName}_deadline_handler(${parameters.joinToString(", ")}); }"
//
//            return if (deadline == null)
//                """
//                    $body
//                    reactor::Reaction $codeName{"$label", $priority, this, [this]() { ${codeName}_body(); }};
//                """.trimIndent()
//            else
//                """
//                    $body
//                    $deadlineHandler
//                    reactor::Reaction $codeName{"$label", $priority, this, [this]() { ${codeName}_body(); }};
//                """.trimIndent()
//        }
//    }
//
//    private fun generateFunctionDeclaration(reaction: Reaction, postfix: String): String {
//        val params = reaction.getBodyParameters()
//        return when (params.size) {
//            0    -> "void ${reaction.codeName}_$postfix();"
//            1    -> "void ${reaction.codeName}_$postfix(${params[0]});"
//            else -> with(PrependOperator) {
//                """
//                    |void ${reaction.codeName}_$postfix(
//                ${" |  "..params.joinToString(",\n")});
//                """.trimMargin()
//            }
//        }
//    }
//
//    private fun getFunctionDefinitionSignature(reaction: Reaction, postfix: String): String {
//        val params = reaction.getBodyParameters()
//        return when (params.size) {
//            0    -> "void ${reactor.templateName}::Inner::${reaction.codeName}_$postfix()"
//            1    -> "void ${reactor.templateName}::Inner::${reaction.codeName}_$postfix(${params[0]})"
//            else -> with(PrependOperator) {
//                """
//                    |void ${reactor.templateName}::Inner::${reaction.codeName}_$postfix(
//                ${" |  "..params.joinToString(",\n")})
//                """.trimMargin()
//            }
//        }
//    }
//
//    private fun generateBodyDefinition(reaction: Reaction): String {
//        return with(PrependOperator) {
//            """
//                |// reaction ${reaction.label}
//                |${reactor.templateLine}
//            ${" |"..getFunctionDefinitionSignature(reaction, "body")} {
//            ${" |  "..reaction.code.toText()}
//                |}
//                |
//            """.trimMargin()
//        }
//    }
//
//    private fun generateDeadlineHandlerDefinition(reaction: Reaction): String = with(PrependOperator) {
//        return """
//            |${reactor.templateLine}
//        ${" |"..getFunctionDefinitionSignature(reaction, "deadline_handler")} {
//        ${" |  "..reaction.deadline.code.toText()}
//            |}
//            |
//        """.trimMargin()
//    }
//
//    private fun generateViewForContainer(r: Reaction, container: Instantiation): String {
//        val reactorClass = with(instanceGenerator) { container.cppType }
//        val viewClass = r.getViewClassName(container)
//        val viewInstance = r.getViewInstanceName(container)
//
//        val variables = r.getAllReferencedVariablesForContainer(container)
//        val instantiations = variables.map {
//            val type = "decltype($reactorClass::${it.variable.name})& ${it.variable.name};"
//            if (it.isEffectOf(r)) type else "const $type"
//        }
//        val initializers = variables.map { "${it.variable.name}(reactor->${it.variable.name})" }
//
//        val viewDeclaration =
//            if (container.isBank) "std::vector<$viewClass> $viewInstance;"
//            else "$viewClass $viewInstance;"
//
//        return with(PrependOperator) {
//            """
//                |struct $viewClass {
//            ${" |  "..instantiations.joinToString("\n")}
//                |  $viewClass($reactorClass* reactor) :
//            ${" |    "..initializers.joinToString(",\n")}
//                |  {}
//                |};
//                |$viewDeclaration
//            """.trimMargin()
//        }
//    }
//
//    private fun generateViews(r: Reaction) =
//        r.allReferencedContainers.joinWithLn { generateViewForContainer(r, it) }
//
//    private fun generateViewInitializers(r: Reaction) =
//        r.allReferencedContainers.filterNot { it.isBank }
//            .joinWithLn { ", ${r.getViewInstanceName(it)}(${it.name}.get()) " }
//
//    private fun generateViewConstructorInitializers(r: Reaction) =
//        r.allReferencedContainers.filter { it.isBank }
//            .joinWithLn {
//                val viewInstance = r.getViewInstanceName(it)
//                """
//                    $viewInstance.reserve(${it.name}.size());
//                    for (auto& __lf_instance : ${it.name}) {
//                      $viewInstance.emplace_back(__lf_instance.get());
//                    }
//                """.trimIndent()
//            }
//
//    fun generateReactionViews() =
//        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") { generateViews(it) }
//
//    fun generateReactionViewInitializers() =
//        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") {
//            generateViewInitializers(it)
//        }
//
//    fun generateReactionViewForwardDeclarations(): String {
//        val classNames = reactor.reactions.map { r -> r.allReferencedContainers.map { r.getViewClassName(it) } }.flatten()
//        if (classNames.isEmpty()) {
//            return ""
//        }
//        return classNames.joinWithLn(prefix = "// reaction view forward declarations\n") { "struct $it;" }
//    }
//
//    fun generateReactionViewConstructorInitializers() =
//        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction views\n", postfix = "\n") {
//            generateViewConstructorInitializers(it)
//        }
//
//    /** Get all reaction declarations. */
//    fun generateDeclarations() =
//        reactor.reactions.joinToString(separator = "\n", prefix = "// reactions\n", postfix = "\n") { generateDeclaration(it) }
//
//    /** Get all declarations of reaction bodies. */
//    fun generateBodyDeclarations() =
//        reactor.reactions.joinToString("\n", "// reaction bodies\n", "\n") {
//            generateFunctionDeclaration(it, "body")
//        }
//
//    /** Get all definitions of reaction bodies. */
//    fun generateBodyDefinitions() =
//        reactor.reactions.joinToString(separator = "\n", postfix = "\n") { generateBodyDefinition(it) }
//
//    /** Get all declarations of deadline handlers. */
//    fun generateDeadlineHandlerDeclarations(): String =
//        reactionsWithDeadlines.joinToString("\n", "// deadline handlers\n", "\n") {
//            generateFunctionDeclaration(it, "deadline_handler")
//        }
//
//    /** Get all definitions of deadline handlers. */
//    fun generateDeadlineHandlerDefinitions() =
//        reactionsWithDeadlines.joinToString(separator = "\n", postfix = "\n") { generateDeadlineHandlerDefinition(it) }
//}
