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

import org.lflang.MessageReporter
import org.lflang.generator.PrependOperator
import org.lflang.lf.BuiltinTrigger
import org.lflang.lf.BuiltinTriggerRef
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.priority
import org.lflang.toUnixString

class UcReactorGenerator(private val reactor: Reactor, fileConfig: UcFileConfig, messageReporter: MessageReporter) {

    private val headerFile = fileConfig.getReactorHeaderPath(reactor).toUnixString()

    private val hasStartup = reactor.reactions.filter {it.triggers.filter {it is BuiltinTriggerRef && it.type == BuiltinTrigger.STARTUP}.isNotEmpty()}.isNotEmpty()
    private val hasShutdown = reactor.reactions.filter {it.triggers.filter {it is BuiltinTriggerRef && it.type == BuiltinTrigger.SHUTDOWN}.isNotEmpty()}.isNotEmpty()
    private fun numTriggers(): Int {
        var res = reactor.actions.size + reactor.timers.size + reactor.inputs.size + reactor.outputs.size;
        if (hasShutdown) res++;
        if (hasStartup) res++;
        return res;
    }
    private val numChildren = reactor.instantiations.size;

//    private val parameters = CppParameterGenerator(reactor)
//    private val state = CppStateGenerator(reactor)
//    private val methods = CppMethodGenerator(reactor)
//    private val instances = CppInstanceGenerator(reactor, fileConfig, messageReporter)
    private val timers = UcTimerGenerator(reactor)
//    private val actions = CppActionGenerator(reactor, messageReporter)
    private val ports = UcPortGenerator(reactor)
    private val reactions = UcReactionGenerator(reactor, ports)

    fun generateReactorStruct() = with(PrependOperator) {
        """
            |typedef struct {
            |  Reactor super;
        ${" |  "..reactions.generateReactorStructFields()}
        ${" |  "..timers.generateReactorStructFields()}
            |  Reaction *_reactions[${reactor.reactions.size}];
            |  Trigger *_triggers[${numTriggers()}];
            |} ${reactor.name};
            """.trimMargin()
    }

    fun generateHeader() = with(PrependOperator) {
        """
            |#include "reactor-uc/reactor-uc.h"
            |
        ${" |"..reactions.generateSelfStructs()}
        ${" |"..timers.generateSelfStructs()}
            | // The reactor self struct
        ${" |"..generateReactorStruct()}
            | // The constructor for the self struct
            |void ${reactor.name}_ctor(${reactor.name} *self, Environment *env, Reactor *parent);
            |
            |
        """.trimMargin()
    }
    fun generateSource() = with(PrependOperator) {
        """
            |#include "${headerFile}"
        ${" |"..reactions.generateReactionBodies()}
        ${" |"..reactions.generateReactionCtors()}
        ${" |"..timers.generateCtors()}
        ${" |"..generateCtorDefinition()}
        """.trimMargin()
    }

    fun generateCtorDefinition() = with(PrependOperator) {
        """
            | void ${reactor.name}_ctor(${reactor.name} *self, Environment *env, Reactor *parent) {
            |  size_t trigger_idx = 0;
            |  Reactor_ctor(&self->super, "${reactor.name}", env, parent, ${if (numChildren > 0) "self->_children" else "NULL"}, $numChildren, ${if (reactor.reactions.size > 0) "self->_reactions" else "NULL"}, ${reactor.reactions.size}, ${if (numTriggers() > 0) "self->_triggers" else "NULL"}, ${numTriggers()});
        ${" |  "..timers.generateReactorCtorCodes()}
        ${" |  "..reactions.generateReactorCtorCodes()}
            | }
        """.trimMargin()
    }
}
