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

/** A code generator for the C++ constructor of a reactor class */
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
    fun generateDeclaration() = "${signature()};"

    /** Get the constructor definition */
    fun generateDefinition(): String {
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
            ${" |  "..state.generateInitializers()}
                |  // TODO «r.initializeInstances»
            ${" |  "..timers.generateInitializers()}
            ${" |  "..actions.geberateInitializers()}
                |{}
            """.trimMargin()
        }
    }
}