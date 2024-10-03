/*************
 * Copyright (c) 2019-2021, TU Dresden.

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

package org.lflang.generator.uc

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.CppInstanceGenerator.Companion.enclaveWrapperClassName
import org.lflang.generator.cpp.CppInstanceGenerator.Companion.isEnclave
import org.lflang.generator.cpp.name
import org.lflang.lf.*

class UcPortGenerator(private val reactor: Reactor) {
    companion object { /** Get the "name" a reaction is represented with in target code.*/
    val Input.codeType
        get(): String = "${(eContainer() as Reactor).name}_Input_$name"
    val Output.codeType
        get(): String = "${(eContainer() as Reactor).name}_Output_$name"
    val Port.codeType
        get(): String = if (this.isInput) (this as Input).codeType else (this as Output).codeType
    }

    fun getEffects(port: Input) = reactor.reactions.filter { it.triggers.filter { it.name == port.name }.isNotEmpty() }
    fun getSources(port: Port) = reactor.reactions.filter { it.effects.filter { it.name == port.name }.isNotEmpty() }

    fun generateSelfStruct(input: Input) = with(PrependOperator) {
        """
            |typedef struct {
            |   Input super;
            |   ${input.type.toText()} buffer[1];
            |   ${if (getEffects(input).size > 0) "Reaction *_effects[${getEffects(input).size}];" else ""}
            |} ${input.codeType};
            
        """.trimMargin()
    }
    fun generateSelfStruct(output: Output) = with(PrependOperator) {
        """
            |typedef struct {
            |   Output super;
            |   ${if (getSources(output).size > 0) "Reaction *_sources[${getSources(output).size}];" else ""}
            |} ${output.codeType};
            
        """.trimMargin()
    }

    fun generateSelfStructs() = reactor.inputs.plus(reactor.outputs).joinToString(prefix = "// Port structs\n", separator = "\n", postfix = "\n") {
        when (it) {
            is Input  -> generateSelfStruct(it)
            is Output -> generateSelfStruct(it)
            else      -> ""
        }
    }

    fun generateReactorStructFields() = reactor.inputs.plus(reactor.outputs).joinToString(prefix = "// Ports \n", separator = "\n", postfix = "\n") {
            "${it.codeType} ${it.name};"
        }

    fun generateInputCtor(input: Input) = with(PrependOperator) {
        """
            |static void ${input.codeType}_ctor(${input.codeType} *self, Reactor *parent) {
            |   Input_ctor(&self->super, parent, ${if (getEffects(input).size > 0) "self->_effects" else "NULL"}, ${getEffects(input).size}, self->buffer, sizeof(self->buffer[0]));
            |}
        """.trimMargin()
    }

    fun generateOutputCtor(output: Output) = with(PrependOperator) {
        """
            |static void ${output.codeType}_ctor(${output.codeType} *self, Reactor *parent) {
            |   Output_ctor(&self->super, parent, ${if (getSources(output).size > 0) "self->_sources" else "NULL"}, ${getSources(output).size});
            |}
        """.trimMargin()
    }
    fun generateCtors() = reactor.inputs.plus(reactor.outputs).joinToString(prefix = "// Port constructors\n", separator = "\n", postfix = "\n") {
        when (it) {
            is Input  -> generateInputCtor(it)
            is Output -> generateOutputCtor(it)
            else      -> ""
        }
    }

    fun generateReactorCtorCode(port: Port)  =  with(PrependOperator) {
        """
            |self->_triggers[trigger_idx++] = (Trigger *) &self->${port.name};
            |${port.codeType}_ctor(&self->${port.name}, &self->super);
            |
            """.trimMargin()
    };
    fun generateReactorCtorCodes() = reactor.inputs.plus(reactor.outputs).joinToString(prefix = "// Initialize ports\n", separator = "\n", postfix = "\n") { generateReactorCtorCode(it)}
}

