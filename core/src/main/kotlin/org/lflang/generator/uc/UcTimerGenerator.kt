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

package org.lflang.generator.uc

import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.name
import org.lflang.generator.cpp.toCppCode
import org.lflang.generator.orZero
import org.lflang.generator.uc.UcReactionGenerator.Companion.bodyFuncName
import org.lflang.generator.uc.UcReactionGenerator.Companion.codeName
import org.lflang.generator.uc.UcReactionGenerator.Companion.codeType
import org.lflang.isGeneric
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.Timer
import org.lflang.priority

class UcTimerGenerator(private val reactor: Reactor) {
    companion object {
        /** Get the "name" a reaction is represented with in target code.*/
        val Timer.codeType
            get(): String = "Timer_${name}"

    }

    fun getEffects(timer: Timer) = reactor.reactions.filter {it.effects.filterNot{ it.name == timer.name}.isEmpty()}

    fun generateEffectsFieldPtr(timer: Timer) = if (getEffects(timer).size > 0) "& self->_effects" else "NULL"

    fun generateSelfStructs(timer: Timer) = with(PrependOperator) {
        """
            |typedef struct {
            |  Timer super;
            |  ${if (getEffects(timer).size > 0) "Reaction *_effects[${getEffects(timer).size}];" else ""}
            |} ${timer.codeType};   
            """.trimMargin()
    };

    fun generateCtors() = reactor.timers.joinToString(
    separator = "\n",
    prefix = "// Timer constructors \n",
    postfix = "\n"
    ) { generateCtor(it) };

    fun generateCtor(timer: Timer) = with(PrependOperator) {
        """
            |static void ${timer.codeType}_ctor(${timer.codeType} *self, Reactor *parent, interval_t offset, interval_t period) {
            |   Timer_ctor(&self->super, parent, offset, period, ${if (getEffects(timer).size > 0) "self->_effects" else "NULL"}, ${getEffects(timer).size});
            |}
        """.trimMargin()
    }

    fun generateSelfStructs() =
        reactor.timers.joinToString(
            separator = "\n",
            prefix = "// Timer structs \n",
            postfix = "\n"
        ) { generateSelfStructs(it) };

    fun generateReactorStructFields() =
        reactor.timers.joinToString(separator = "\n", prefix = "// timers\n", postfix = "\n") { "${it.codeType} ${it.name};" }

    fun generateReactorCtorCode(timer: Timer)  =  with(PrependOperator) {
        """
            |self->_triggers[trigger_idx++] = &self->${timer.name}.super.super;
            |${timer.codeType}_ctor(&self->${timer.name}, &self->super, ${timer.offset.toCCode()}, ${timer.period.toCCode()});
            |
            """.trimMargin()
    };

    fun generateReactorCtorCodes() = reactor.timers.joinToString(separator = "\n", prefix = "// Timers \n") { generateReactorCtorCode(it)}
}
