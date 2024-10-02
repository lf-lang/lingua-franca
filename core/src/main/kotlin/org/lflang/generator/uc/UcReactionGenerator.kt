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

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.name
import org.lflang.generator.uc.UcActionGenerator.Companion.codeType
import org.lflang.generator.uc.UcPortGenerator.Companion.codeType
import org.lflang.generator.uc.UcTimerGenerator.Companion.codeType
import org.lflang.lf.*

class UcReactionGenerator(
    private val reactor: Reactor,
    private val portGenerator: UcPortGenerator
) {
    companion object { /** Get the "name" a reaction is represented with in target code.*/
        val Reaction.codeType
            get(): String = name ?: "Reaction_$priority"
        val Reaction.codeName
            get(): String = name ?: "reaction_$priority"
        val Reaction.bodyFuncName
            get(): String = name ?: "Reaction_${priority}_body"
    }

    fun generateEffectsField(reaction: Reaction) =
        if (reaction.allUncontainedEffects.size > 0) "Trigger *_effects[${reaction.allUncontainedEffects.size}]" else "\n"

    fun generateEffectsFieldPtr(reaction: Reaction) = if (reaction.allUncontainedEffects.size > 0) "& self->_effects" else "NULL"

    fun generateReactionCtor(reaction: Reaction) = with(PrependOperator) {
        """
        |static void ${reaction.codeType}_ctor(${reaction.codeType} *self, Reactor *parent) {
        |   Reaction_ctor(&self->super, parent, ${reaction.bodyFuncName}, ${generateEffectsFieldPtr(reaction)}, ${reaction.allUncontainedEffects.size}, ${reaction.priority-1});
        |}
       """.trimMargin()
    };

    fun generateSelfStructs(reaction: Reaction) = with(PrependOperator) {
        """
            |typedef struct {
            |  Reaction super;
        ${" |  "..generateEffectsField(reaction)}
            |} ${reaction.codeType};
            """.trimMargin()
    };

    fun generateSelfStructs() =
        reactor.reactions.joinToString(
            separator = "\n",
            prefix = "// Reaction self-structs and constructors \n",
            postfix = "\n"
        ) { generateSelfStructs(it) };

    fun generateReactorStructFields() =
        reactor.reactions.joinToString(
            separator = "\n",
            postfix = "\n"
        ) {
            with(PrependOperator) {
                """
                |${it.codeType} ${it.codeName};
            """.trimMargin()
            }
        };

    fun generateReactionCtors() =
        reactor.reactions.joinToString(
            separator = "\n",
            prefix = "// Reaction self-struct constructors \n",
            postfix = "\n"
        ) { generateReactionCtor(it) };

    fun generateReactionBodies() =
        reactor.reactions.joinToString(
            separator = "\n",
            prefix = "// Reaction bodies\n",
            postfix = "\n"
        ) { generateReactionBody(it) };

    fun generateReactionBody(reaction: Reaction) = with(PrependOperator) {
        """
            |static void ${reaction.bodyFuncName}(Reaction *_self) {
            |   // Bring expected variable names into scope
            |   ${reactor.name} *self = (${reactor.name} *) _self->parent;
            |   Environment *env = self->super.env;
            |   ${generateTriggersInScope(reaction)}
            |   // Start of user-witten reaction body
            |   ${reaction.code.toText()}
            |   // End of user-written reaction body
            |}
        """.trimMargin()
    }

    fun generateTriggersInScope(reaction: Reaction) =
        reaction.allUncontainedTriggers.plus(reaction.allUncontainedTriggers).joinToString(separator = "/n"){
            when(it) {
                is Action -> "${it.codeType} *${it.name} = &self->${it.name}"
                is Input -> "${it.codeType} *${it.name} = &self->${it.name}"
                is Output -> "${it.codeType} *${it.name} = &self->${it.name}"
                is Timer -> "${it.codeType} *${it.name} = &self->${it.name}"
                is BuiltinTriggerRef-> "${it.codeType} *${it.name} = &self->${it.name}"
                else -> ""
            }
        }

    fun generateTriggerRegisterEffect(reaction: Reaction) =
        reaction.allUncontainedTriggers.joinToString(
            separator = "\n",
            postfix = "\n"
        ) {
            when(it) {
            is Timer -> "TIMER_REGISTER_EFFECT(self->${it.name}, self->${reaction.name}"
            is Action -> "ACTION_REGISTER_EFFECT(self->${it.name}, self->${reaction.name}"
            is Input -> "INPUT_REGISTER_EFFECT(self->${it.name}, self->${reaction.name}"
            is BuiltinTriggerRef -> {
                if (it.type == BuiltinTrigger.STARTUP) {
                     "STARTUP_REGISTER_EFFECT(self->${it.name}, self->${reaction.name}"
                } else if(it.type == BuiltinTrigger.SHUTDOWN) {
                     "SHUTDOWN_REGISTER_EFFECT(self->${it.name}, self->${reaction.name}"
                } else {
                    ""
                }
            }
                else -> ""
            }
            };

    fun generateTriggerRegisterSource(reaction: Reaction) =
        reaction.allUncontainedEffects.joinToString(
            separator = "\n",
            postfix = "\n"
            ) {
                when(it) {
                    is Action -> "ACTION_REGISTER_SOURCE(self->${it.name}, self->${reaction.name}"
                    is Output -> "OUTPUT_REGISTER_SOURCE(self->${it.name}, self->${reaction.name}"
                else -> ""
                    }
            };

    fun generateRegisterEffects(reaction: Reaction) =
        reaction.allUncontainedEffects.joinToString(
            separator = "\n",
            postfix = "\n"
        ) {
            "self->super.register_effect(&self->super, (Trigger *)&self->${it.name});"
        };

    fun generateReactorCtorCode(reaction: Reaction) = with(PrependOperator) {
        """
            |self->_reactions[${reaction.priority - 1}] = &self->${reaction.codeName}.super;
            |${reaction.codeType}_ctor(&self->${reaction.codeName}, &self->super);
            |// Register all triggers of this reaction.
        ${" |"..generateTriggerRegisterEffect(reaction)}
        ${" |"..generateTriggerRegisterSource(reaction)}
        ${" |"..generateRegisterEffects(reaction)}
            """.trimMargin()
    };

    fun generateReactorCtorCodes() =
        reactor.reactions.joinToString(separator = "\n", prefix = "// Reactions \n") { generateReactorCtorCode(it) }

    private val VarRef.isContainedRef: Boolean get() = container != null
        private val TriggerRef.isContainedRef: Boolean get() = this is VarRef && isContainedRef

    private fun VarRef.isEffectOf(reaction: Reaction): Boolean =
        reaction.effects.any { name == it.name && container?.name == it.container?.name }

    private fun TriggerRef.isEffectOf(reaction: Reaction): Boolean = this is VarRef && isEffectOf(reaction)

    private val Reaction.allUncontainedTriggers get() = triggers.filterNot { it.isEffectOf(this) || it.isContainedRef }
    private val Reaction.allUncontainedEffects get() = effects.filterNot { it.isContainedRef };

}
