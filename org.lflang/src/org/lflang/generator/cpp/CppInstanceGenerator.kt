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

import org.lflang.lf.Instantiation
import org.lflang.lf.Reactor

/** A code genarator for reactor instances */
class CppInstanceGenerator(private val reactor: Reactor, private val fileConfig: CppFileConfig) {

    private val Instantiation.type: String
        get() {
            return if (this.reactor.isGeneric)
                """${this.reactor.name}<${this.typeParms.joinToString(", ")}>}"""
            else
                this.reactor.name
        }

    private fun generateDeclaration(inst: Instantiation): String {
        return if (inst.isBank)
            "std::array<std::unique_ptr<${inst.type}>, ${inst.width}> ${inst.name};"
        else
            "std::unique_ptr<${inst.type}> ${inst.name};"
    }

    private fun generateInitializer(inst: Instantiation): String {
        return if (inst.isBank) {
            val initializations = (0 until inst.width).map {
                """std::make_unique<${inst.type}>("${inst.name}_$it", this, $it)"""
            }.joinToString(", ")
            """, ${inst.name}{{$initializations}}"""
        } else {
            """, ${inst.name}(std::make_unique<${inst.type}>("${inst.name}", this))"""
        }

        // TODO Support parameters
        /*
        def initializerList(Instantiation i) '''
            "«i.name»", this«FOR p : i.reactorClass.toDefinition.parameters», «p.getTargetInitializer(i)»«ENDFOR»
        '''

        def initializerList(Instantiation i, Integer id) '''
            "«i.name»_«id»", this«FOR p : i.reactorClass.toDefinition.parameters», «IF p.name == "instance"»«id»«ELSE»«p.getTargetInitializer(i)»«ENDIF»«ENDFOR»
        '''

        def initializeInstances(Reactor r) '''
            «FOR i : r.instantiations BEFORE "// reactor instantiations \n"»
                «IF i.widthSpec !== null»
                    , «i.name»{{«FOR id : IntStream.range(0, i.widthSpecification).toArray SEPARATOR ", "»std::make_unique<«i.instanceType»>(«i.initializerList(id)»)«ENDFOR»}}
                «ELSE»
                    , «i.name»(std::make_unique<«i.instanceType»>(«i.initializerList»))
                «ENDIF»
            «ENDFOR»
        '''
         */
    }

    /** Generate C++ include statements for each reactor that is instantiated */
    fun generateIncludes(): String =
        reactor.instantiations.map { fileConfig.getReactorHeaderPath(it.reactor) }
            .distinct()
            .joinToString(separator = "\n") { """#include "${it.toUnixString()}" """ }

    /** Generate declaration statements for all reactor instantiations */
    fun generateDeclarations(): String {
        // FIXME: Does not support parameter values for widths.
        return reactor.instantiations.joinToString(prefix = "// reactor instances\n", separator = "\n") { generateDeclaration(it) }
    }

    /** Generate constructor initializers for all reactor instantiations */
    // FIXME: Does not support parameter values for widths.
    fun generateInitializers(): String =
        reactor.instantiations.joinToString(prefix = "//reactor instances\n", separator = "\n") { generateInitializer(it) }
}
