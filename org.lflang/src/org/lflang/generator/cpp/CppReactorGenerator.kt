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

import org.lflang.lf.Reactor

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

    /** Generate a C++ header file declaring the given reactor. */
    fun header() = """
    ${" |"..fileComment}
        | 
        |#pragma once
        |
        |#include "reactor-cpp/reactor-cpp.hh"
        |
        |#include "$preambleHeaderFile"
        |
        |// TODO «r.includeInstances»
        |// TODO «r.publicPreamble»
        |
        |// TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
        |class ${reactor.name} : public reactor::Reactor {
        | private:
        |  // TODO «r.declareParameters»
        |  // TODO «r.declareStateVariables»
        |  // TODO «r.declareInstances»
        |  // TODO «r.declareTimers»
        |  // TODO «r.declareActions»
        |  // TODO «r.declareReactions»
        |  // TODO «r.declareReactionBodies»
        |  // TODO «r.declareDeadlineHandlers»
        | public:
        |  // TODO «r.declarePorts»
        |  // TODO declareConstructor(r)
        |
        |  void assemble() override;
        |};
        |/* TODO
        |«IF r.isGeneric»
        |
        |#include "«r.headerImplFile.toUnixString»"
        |«ENDIF»*/
    """.trimMargin()

    /** Generate a C++ source file implementing the given reactor. */
    fun source() = """
    ${" |"..fileComment}
        |
        |${if (!reactor.isGeneric) """#include "$headerFile"""" else ""}
        |#include "lfutil.hh"
        |
        |using namespace std::chrono_literals;
        |using namespace reactor::operators;
        |
        |// TODO «r.privatePreamble»
        |
        |// TODO «r.defineConstructor»
        |
        |// «r.defineAssembleMethod»
        |
        |// TODO «r.implementReactionBodies»
        |// TODO «r.implementReactionDeadlineHandlers»
    """.trimMargin()
}