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

import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.toText

/** A C++ code generator for reactions and their function bodies */
class CppReactionGenerator(private val reactor: Reactor) {

    private val reactionsWithDeadlines = reactor.reactions.filter { it.deadline != null }

    private fun generateDeclaration(r: Reaction) =
        """reactor::Reaction ${r.name}{"${r.label}", ${r.priority}, this, [this]() { ${r.name}_body(); }};"""

    private fun generateBodyDefinition(reaction: Reaction) = with(prependOperator) {
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

    private fun generateDeadlineHandlerDefinition(reaction: Reaction): String = with(prependOperator) {
        // TODO Should provide the same context as in reactions
        // TODO «IF r.isGeneric»«r.templateLine»«ENDIF»
        return """
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
        reactor.reactions.joinToString("\n", "// reaction bodies\n", "\n") { "void ${it.name}_body();" }

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