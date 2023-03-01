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

import org.lflang.ErrorReporter
import org.lflang.generator.PrependOperator
import org.lflang.isGeneric
import org.lflang.lf.Reactor
import org.lflang.toText
import org.lflang.toUnixString

/**
 * A C++ code generator that produces a C++ class representing a single reactor
 */
class CppReactorGenerator(private val reactor: Reactor, fileConfig: CppFileConfig, errorReporter: ErrorReporter) {

    /** Comment to be inserted at the top of generated files */
    private val fileComment = fileComment(reactor.eResource())

    /** The header file that declares `reactor` */
    private val headerFile = fileConfig.getReactorHeaderPath(reactor).toUnixString()

    /** The implementation header file that declares a `reactor` if it is generic*/
    private val implHeaderFile = fileConfig.getReactorHeaderImplPath(reactor).toUnixString()

    /** The header file that contains the public file-level preamble of the file containing `reactor` */
    private val preambleHeaderFile = fileConfig.getPreambleHeaderPath(reactor.eResource()).toUnixString()

    private val parameters = CppParameterGenerator(reactor)
    private val state = CppStateGenerator(reactor)
    private val methods = CppMethodGenerator(reactor)
    private val instances = CppInstanceGenerator(reactor, fileConfig, errorReporter)
    private val timers = CppTimerGenerator(reactor)
    private val actions = CppActionGenerator(reactor, errorReporter)
    private val ports = CppPortGenerator(reactor)
    private val reactions = CppReactionGenerator(reactor, ports, instances)
    private val assemble = CppAssembleMethodGenerator(reactor)
    private val connections = CppConnectionGenerator(reactor)

    private fun publicPreamble() =
        reactor.preambles.filter { it.isPublic }
            .joinToString(separator = "\n", prefix = "// public preamble\n") { it.code.toText() }

    private fun privatePreamble() =
        reactor.preambles.filter { it.isPrivate }
            .joinToString(separator = "\n", prefix = "// private preamble\n") { it.code.toText() }

    /** Generate a C++ header file declaring the given reactor. */
    fun generateHeader() = with(PrependOperator) {
        """
        ${" |"..fileComment}
            | 
            |#pragma once
            |
            |#include "reactor-cpp/reactor-cpp.hh"
            |#include "lfutil.hh"
            |
            |using namespace std::chrono_literals;
            |
            |#include "$preambleHeaderFile"
            |
        ${" |"..instances.generateIncludes()}
            |
        ${" |"..publicPreamble()}
            |
            |${reactor.templateLine}
            |class ${reactor.name}: public reactor::Reactor {
            |public:
            |  struct Parameters {
        ${" |    "..parameters.generateParameterStructDeclarations()}
            |  };
            |
            | private:
        ${" |  "..reactions.generateReactionViewForwardDeclarations()}
            |
            |  class Inner: public lfutil::LFScope, public Parameters {
        ${" |    "..parameters.generateUsingDeclarations()}
        ${" |    "..state.generateDeclarations()}
        ${" |    "..methods.generateDeclarations()}
        ${" |    "..reactions.generateBodyDeclarations()}
        ${" |    "..reactions.generateDeadlineHandlerDeclarations()}
            |
            |    Inner(reactor::Reactor* reactor, Parameters&& parameters);
            |
            |   friend ${reactor.name};
            |  };
            |
            |  Inner __lf_inner;
            |
        ${" |  "..parameters.generateOuterAliasDeclarations()}
        ${" |  "..instances.generateDeclarations()}
        ${" |  "..timers.generateDeclarations()}
        ${" |  "..actions.generateDeclarations()}
        ${" |  "..reactions.generateReactionViews()}
        ${" |  "..reactions.generateDeclarations()}
            |
            | public:
        ${" |  "..ports.generateDeclarations()}
        ${" |  "..outerConstructorSignature(true)};
        ${" |  "..outerConstructorSignature(false)};
            |
            |  void assemble() override;
            | 
            | private:
        ${" |  "..connections.generateDeclarations()}
            |};
            |
        ${" |"..if (reactor.isGeneric) """#include "$implHeaderFile"""" else ""}
        """.trimMargin()
    }

    /** Generate a C++ source file implementing the given reactor. */
    fun generateSource() = with(PrependOperator) {
        """
        ${" |"..fileComment}
            |
            |${if (!reactor.isGeneric) """#include "$headerFile"""" else ""}
            |
            |using namespace reactor::operators;
            |
        ${" |  "..privatePreamble()}
            |
            |// outer constructor
        ${" |"..generateOuterConstructorDefinition(true)}
        ${" |"..generateOuterConstructorDefinition(false)}
            |
            |// inner constructor
        ${" |"..generateInnerConstructorDefinition()}
            |
        ${" |"..assemble.generateDefinition()}
            |
        ${" |"..methods.generateDefinitions()}
            |
        ${" |"..reactions.generateBodyDefinitions()}
        ${" |"..reactions.generateDeadlineHandlerDefinitions()}
        """.trimMargin()
    }

    private fun generateInnerConstructorDefinition(): String {
        return with(PrependOperator) {
            """
                |${reactor.templateLine}
                |${reactor.templateName}::Inner::Inner(::reactor::Reactor* reactor, Parameters&& __lf_parameters)
                |  : LFScope(reactor)
            ${" |  , Parameters(std::forward<Parameters>(__lf_parameters))"}
            ${" |  "..state.generateInitializers()}
                |{}
                """.trimMargin()
        }
    }

    private fun outerConstructorSignature(fromEnvironment: Boolean): String {
        val containerRef = if (fromEnvironment) "reactor::Environment* __lf_environment" else "reactor::Reactor* __lf_container"
        return "${reactor.name}(const std::string& name, $containerRef, Parameters&& __lf_parameters)"
    }

    /** Get the constructor definition of the outer reactor class */
    private fun generateOuterConstructorDefinition(fromEnvironment: Boolean): String {
        return with(PrependOperator) {
            """
                |${reactor.templateLine}
                |${reactor.templateName}::${outerConstructorSignature(fromEnvironment)}
                |  : reactor::Reactor(name, ${if (fromEnvironment) "__lf_environment" else "__lf_container"})
                |  , __lf_inner(this, std::forward<Parameters>(__lf_parameters))
            ${" |  "..instances.generateInitializers()}
            ${" |  "..timers.generateInitializers()}
            ${" |  "..actions.generateInitializers()}
            ${" |  "..connections.generateInitializers()}
            ${" |  "..reactions.generateReactionViewInitializers()}
                |{
            ${" |  "..ports.generateConstructorInitializers()}
            ${" |  "..instances.generateConstructorInitializers()}
            ${" |  "..reactions.generateReactionViewConstructorInitializers()}
                |}
            """.trimMargin()
        }
    }
}

