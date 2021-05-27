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

import org.lflang.isInitialized
import org.lflang.isOfTimeType
import org.lflang.lf.*
import org.lflang.toText

/**
 * A class responsible for generating code for a single given reactor.
 */
class CppReactorGenerator(private val reactor: Reactor, private val fileConfig: CppFileConfig) {

    /** Comment to be inserted at the top of generated files */
    private val fileComment = fileComment(reactor.eResource())

    /** The header file that declares `reactor` */
    private val headerFile = fileConfig.getReactorHeaderPath(reactor).toUnixString()

    /** The header file that contains the public file-level preamble of the file containing `reactor` */
    private val preambleHeaderFile = fileConfig.getPreambleHeaderPath(reactor.eResource()).toUnixString()

    private val state = CppStateGenerator(reactor)
    private val timers = CppTimerGenerator(reactor)
    private val actions = CppActionGenerator(reactor)
    private val reactions = CppReactionGenerator(reactor)
    private val constructor = CppConstructorGenerator(reactor, state, timers, actions)
    private val assemble = CppAssembleMethodGenerator(reactor)

    private fun publicPreamble() =
        reactor.preambles.filter { it.isPublic }
            .joinToString(separator = "\n", prefix = "// public preamble\n") { it.code.toText() }

    private fun privatePreamble() =
        reactor.preambles.filter { it.isPrivate }
            .joinToString(separator = "\n", prefix = "// private preamble\n") { it.code.toText() }

    /** Generate a C++ header file declaring the given reactor. */
    fun header() = with(prependOperator) {
        """
        ${" |"..fileComment}
            | 
            |#pragma once
            |
            |#include "reactor-cpp/reactor-cpp.hh"
            |
            |#include "$preambleHeaderFile"
            |
            |// TODO «r.includeInstances»
        ${" |  "..publicPreamble()}
            |
            |// TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
            |class ${reactor.name} : public reactor::Reactor {
            | private:
            |  // TODO «r.declareParameters»
        ${" |  "..state.declarations()}
            |  // TODO «r.declareInstances»
        ${" |  "..timers.declarations()}
        ${" |  "..actions.declarations()}
        ${" |  "..reactions.declarations()}
        ${" |  "..reactions.bodyDeclarations()}
            |  // TODO «r.declareDeadlineHandlers»
            | public:
            |  // TODO «r.declarePorts»
        ${" |  "..constructor.declaration()}
            |
            |  void assemble() override;
            |};
            |/* TODO
            |«IF r.isGeneric»
            |
            |#include "«r.headerImplFile.toUnixString»"
            |«ENDIF»*/
        """.trimMargin()
    }

    /** Generate a C++ source file implementing the given reactor. */
    fun source() = with(prependOperator) {
        """
        ${" |"..fileComment}
            |
            |${if (!reactor.isGeneric) """#include "$headerFile"""" else ""}
            |#include "lfutil.hh"
            |
            |using namespace std::chrono_literals;
            |using namespace reactor::operators;
            |
        ${" |  "..privatePreamble()}
            |
        ${" |"..constructor.definition()}
            |
        ${" |"..assemble.definition()}
            |
        ${" |"..reactions.bodyDefinitions()}
            |// TODO «r.implementReactionDeadlineHandlers»
        """.trimMargin()
    }
}

class CppConstructorGenerator(
    private val reactor: Reactor,
    private val state: CppStateGenerator,
    private val timers: CppTimerGenerator,
    private val actions: CppActionGenerator
) {

    /**
     * Constructor argument that provides a reference to the next higher level
     *
     * For the main reactor, the next higher level in the environment. For all other reactors, it is the containing reactor.
     */
    private val environmentOrContainer =
        if (reactor.isMain) "reactor::Environment* environment" else "reactor::Reactor* container"

    private fun signature(): String {
        if (reactor.parameters.size > 0) {
            TODO("Main reactors with parameters are not supported yet")
            /*
            «r.name»(const std::string& name,
            «IF r == mainReactor»reactor::Environment* environment«ELSE»reactor::Reactor* container«ENDIF»,
            «FOR p : r.parameters SEPARATOR ",\n" AFTER ");"»std::add_lvalue_reference<std::add_const<«p.targetType»>::type>::type «p.name» = «p.targetInitializer»«ENDFOR»
            */
            /*  // from constructorDefinition()
            «IF r.isGeneric»«r.templateLine»«ENDIF»
            «IF r.parameters.length > 0»
            «r.templateName»::«r.name»(const std::string& name,
            «IF r == mainReactor»reactor::Environment* environment«ELSE»reactor::Reactor* container«ENDIF»,
            «FOR p : r.parameters SEPARATOR ",\n" AFTER ")"»std::add_lvalue_reference<std::add_const<«p.targetType»>::type>::type «p.name»«ENDFOR»
            «ELSE»
         */
        } else {
            return "${reactor.name}(const std::string& name, $environmentOrContainer)"
        }
    }

    /** Get the constructor declaration */
    fun declaration() = "${signature()};"

    /** Get the constructor definition */
    fun definition(): String {
        /*  TODO
            «IF r.isGeneric»«r.templateLine»«ENDIF»
            «IF r.parameters.length > 0»
            «r.templateName»::«r.name»(const std::string& name,
            «IF r == mainReactor»reactor::Environment* environment«ELSE»reactor::Reactor* container«ENDIF»,
            «FOR p : r.parameters SEPARATOR ",\n" AFTER ")"»std::add_lvalue_reference<std::add_const<«p.targetType»>::type>::type «p.name»«ENDFOR»
            «ELSE»
         */
        return with(prependOperator) {
            """
                |${reactor.name}::${signature()}
                |  : reactor::Reactor(name, ${if (reactor.isMain) "environment" else "container"})
                |  // TODO «r.initializeParameters»
            ${" |  "..state.initializers()}
                |  // TODO «r.initializeInstances»
            ${" |  "..timers.initializers()}
            ${" |  "..actions.initializers()}
                |{}
            """.trimMargin()
        }
    }
}

class CppAssembleMethodGenerator(private val reactor: Reactor) {

    private fun declareTrigger(reaction: Reaction, trigger: TriggerRef): String {
        // check if the trigger is a multiport
        if (trigger is VarRef && trigger.variable is Port) {
            val port = trigger.variable as Port
            if (port.widthSpec != null) {
                return """
                    for (unsigned i = 0; i < ${trigger.name}.size(); i++) {
                      ${reaction.name}.declare_trigger(&${trigger.name}[i]);
                    }
                """.trimIndent()
            }
        }
        // treat as single trigger otherwise
        return "${reaction.name}.declare_trigger(&${trigger.name});"
    }

    private fun declareDependency(reaction: Reaction, dependency: VarRef): String {
        val variable = dependency.variable
        // check if the dependency is a multiport
        if (variable is Port && variable.widthSpec != null) {
            return """
                for (unsigned i = 0; i < ${dependency.name}.size(); i++) {
                  ${reaction.name}.declare_dependency(&${dependency.name}[i]);
                }
            """.trimIndent()
        }
        // treat as single dependency otherwise
        return "${reaction.name}.declare_dependency(&${dependency.name});"
    }

    private fun declareAntidependency(reaction: Reaction, antidependency: VarRef): String {
        val variable = antidependency.variable
        // check if the dependency is a multiport
        if (variable is Port && variable.widthSpec != null) {
            return """
                for (unsigned i = 0; i < ${antidependency.name}.size(); i++) {
                  ${reaction.name}.declare_antidependency(&${antidependency.name}[i]);
                }
            """.trimIndent()
        }
        // treat as single antidependency otherwise
        return if (variable is Action) "${reaction.name}.declare_scheduable_action(&${antidependency.name});"
        else "${reaction.name}.declare_antidependency(&${antidependency.name});"
    }

    private fun assembleReaction(reaction: Reaction) = with(prependOperator) {
        """
            |// ${reaction.name}
        ${" |"..reaction.triggers.joinToString(separator = "\n") { declareTrigger(reaction, it) }}
        ${" |"..reaction.sources.joinToString(separator = "\n") { declareDependency(reaction, it) }}
        ${" |"..reaction.effects.joinToString(separator = "\n") { declareAntidependency(reaction, it) }}
            |// TODO «IF n.deadline !== null»
            |// TODO «n.name».set_deadline(«n.deadline.delay.targetTime», [this]() { «n.name»_deadline_handler(); });
            |// TODO «ENDIF»
        """.trimMargin()
    }

    /**
     * Generate the definition of the reactor's assemble() method
     *
     * The body of this method will declare all triggers, dependencies and antidependencies to the runtime.
     */
    fun definition() = with(prependOperator) {
        """
            |// TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
            |void ${reactor.templateName}::assemble() {
        ${" |  "..reactor.reactions.joinToString(separator = "\n\n") { assembleReaction(it) }}
            |  // TODO «FOR c : r.connections BEFORE "  // connections\n"»
            |  // TODO «c.generate»
            |  // TODO «ENDFOR»
            |}
        """.trimMargin()
    }
}

class CppReactionGenerator(private val reactor: Reactor) {

    private fun declaration(r: Reaction) =
        """reactor::Reaction ${r.name}{"${r.label}", ${r.priority}, this, [this]() { ${r.name}_body(); }};"""

    private fun bodyDeclaration(r: Reaction) = "void ${r.name}_body();"

    private fun bodyDefinition(reaction: Reaction) = with(prependOperator) {
        """
            |// reaction ${reaction.label}
            |// TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
            |void ${reactor.templateName}::${reaction.name}_body() {
            |  // prepare scope
            |  /* TODO
            |     «FOR i : r.instantiations»
            |       «IF i.widthSpec === null»auto& «i.name» = *(this->«i.name»);«ENDIF»
            |    «ENDFOR»
            |  */
            |
            |  // reaction code
        ${" |  "..reaction.code.toText()}
            |}
            |
        """.trimMargin()
    }

    /** Get all reaction declarations. */
    fun declarations() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reactions\n", postfix = "\n") { declaration(it) }

    /** Get all declarations of reaction bodies. */
    fun bodyDeclarations() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// reaction bodies\n", postfix = "\n") { bodyDeclaration(it) }

    fun bodyDefinitions() =
        reactor.reactions.joinToString(separator = "\n", postfix = "\n") { bodyDefinition(it) }
}

class CppActionGenerator(private val reactor: Reactor) {

    private val Action.cppType get() = if (this.isLogical) "reactor::LogicalAction" else "reactor::PhysicalAction"

    private val startupName = LfPackage.Literals.TRIGGER_REF__STARTUP.name
    private val shutdownName = LfPackage.Literals.TRIGGER_REF__SHUTDOWN.name

    private fun declaration(action: Action) = "${action.cppType}<${action.targetType}> ${action.name};"

    private fun initialize(action: Action) = if (action.isLogical) initializeLogical(action) else initializePhysical(action)

    private fun initializeLogical(action: Action): String {
        return if (action.minSpacing != null || !action.policy.isNullOrEmpty()) {
            TODO("How to report errors from here?")
            //action.reportError(
            //    "minSpacing and spacing violation policies are not yet supported for logical actions in reactor-ccp!");
        } else if (action.minDelay != null) {
            """, ${action.name}{"${action.name}", this, ${action.minDelay.time.toCode()}}"""
        } else {
            """, ${action.name}{"${action.name}", this}"""
        }
    }

    private fun initializePhysical(action: Action): String {
        return if (action.minDelay != null || action.minSpacing != null || action.policy.isNullOrEmpty()) {
            TODO("How to report errors from here?")
            //a.reportError(
            //"minDelay, minSpacing and spacing violation policies are not yet supported for physical actions in reactor-ccp!");
        } else {
            """, ${action.name}{"${action.name}", this}"""
        }
    }

    /** Get all action declarations */
    fun declarations() = with(prependOperator) {
        """
        ${" |"..reactor.actions.joinToString(separator = "\n", prefix = "// actions\n", postfix = "\n") { declaration(it) }}
            |// default actions
            |reactor::StartupAction $startupName {"$startupName", this};
            |reactor::ShutdownAction $shutdownName {"$shutdownName", this};
        """.trimMargin()
    }

    /** Get all action initializiers */
    fun initializers() =
        reactor.actions.joinToString(separator = "\n", prefix = "// actions\n", postfix = "\n") { initialize(it) }
}

class CppTimerGenerator(private val reactor: Reactor) {

    private fun initialize(timer: Timer): String {
        val offset = timer.offset?.toTime() ?: "reactor::Duration::zero()"
        val period = timer.period?.toTime() ?: "reactor::Duration::zero()"
        return """${timer.name}{"${timer.name}", this, $period, $offset}"""
    }

    /** Get all timer declarations */
    fun declarations() =
        reactor.timers.joinToString(separator = "\n", prefix = "// timers\n", postfix = "\n") { "reactor::Timer ${it.name};" }

    /** Get all timer initializers */
    fun initializers() =
        reactor.timers.joinToString(separator = "\n", prefix = "// timers\n") { ", ${initialize(it)}" }
}

class CppStateGenerator(private val reactor: Reactor) {

    /**
     * Create a list of state initializers in target code.
     *
     * TODO This is redundant to GeneratorBase.getInitializerList
     */
    private fun getInitializerList(state: StateVar) = state.init.map {
        when {
            it.parameter != null -> it.parameter.name
            state.isOfTimeType   -> it.toTime()
            else                 -> it.toCode()
        }
    }

    private fun initialize(state: StateVar): String {
        return "${state.name}{${getInitializerList(state).joinToString(separator = ", ")}}"
    }

    /** Get all state declarations */
    fun declarations() =
        reactor.stateVars.joinToString(
            separator = "\n",
            prefix = "// state variable\n",
            postfix = "\n"
        ) { "${it.targetType} ${it.name};" }

    /** Get all timer initializers */
    fun initializers(): String = reactor.stateVars.filter { it.isInitialized }
        .joinToString(separator = "\n", prefix = "// state variables\n") { ", ${initialize(it)}" }
}