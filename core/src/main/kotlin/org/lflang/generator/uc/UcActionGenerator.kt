package org.lflang.generator.uc

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.cpp.name
import org.lflang.generator.orZero
import org.lflang.generator.uc.UcTimerGenerator.Companion.codeType
import org.lflang.lf.*

class UcActionGenerator(private val reactor: Reactor) {
    val BuiltinTrigger.codeType
        get(): String =
            if (this == BuiltinTrigger.STARTUP) {
                "${reactor.name}_Startup"
            } else if (this == BuiltinTrigger.SHUTDOWN) {
                "${reactor.name}_Shutdown"
            } else {
                assert(false)
                ""
            }
    val BuiltinTrigger.codeName
        get(): String =
            if (this == BuiltinTrigger.STARTUP) {
                "startup"
            } else if (this == BuiltinTrigger.SHUTDOWN) {
                "shutdown"
            } else {
                unreachable()
            }

    val Action.codeType
        get(): String = "${reactor.name}_Action_$name"

    val Action.bufSize
        get(): Int = 1 // FIXME: This is a parameter/annotation

    private val hasStartup = reactor.reactions.filter {
        it.triggers.filter { it is BuiltinTriggerRef && it.type == BuiltinTrigger.STARTUP }.isNotEmpty()
    }.isNotEmpty()
    private val hasShutdown = reactor.reactions.filter {
        it.triggers.filter { it is BuiltinTriggerRef && it.type == BuiltinTrigger.SHUTDOWN }.isNotEmpty()
    }.isNotEmpty()

    fun getEffects(action: Action) = reactor.reactions.filter { it.triggers.filter { it.name == action.name }.isNotEmpty() }
    fun getSources(action: Action) = reactor.reactions.filter { it.effects.filter { it.name == action.name }.isNotEmpty() }

    fun getEffects(builtinTrigger: BuiltinTrigger) =
        reactor.reactions.filter { it.triggers.filter { it.name == builtinTrigger.literal}.isNotEmpty() }

    fun generateSelfStruct(action: Action) = with(PrependOperator) {
        """
            |typedef struct {
            |  ${if (action.isLogical) "LogicalAction" else "PhysicalAction"} super;
            |  ${action.type.code.toText()} buffer[${action.bufSize}];
            |  ${if (getEffects(action).size > 0) "Reaction *_effects[${getEffects(action).size}];" else ""}
            |  ${if (getSources(action).size > 0) "Reaction *_sources[${getSources(action).size}];" else ""}
            |} ${action.codeType};   
            """.trimMargin()
    };

    fun generateCtor(action: Action) = with(PrependOperator) {
        """
            |static void ${action.codeType}_ctor(${action.codeType} *self, Reactor *parent, interval_t min_delay, interval_t min_spacing) {
            |   ${if (action.isLogical) "Logical" else "Physical"}Action_ctor(&self->super, parent, min_delay, min_spacing, parent, ${if (getSources(action).size > 0) "self->_sources" else "NULL"}, ${getSources(action).size}, ${if (getEffects(action).size > 0) "self->_effects" else "NULL"}, ${getEffects(action).size}, self->buffer, sizeof(self->buffer[0]), ${action.bufSize});
            |}
        """.trimMargin()
    }
    fun generateCtor(builtin: BuiltinTrigger) = with(PrependOperator) {
        """
            |static void ${builtin.codeType}_ctor(${builtin.codeType} *self, Reactor *parent) {
            |   ${if (builtin == BuiltinTrigger.STARTUP) "Startup" else "Shutdown"}_ctor(&self->super, parent, ${if (getEffects(builtin).size > 0) "self->_effects" else "NULL"}, ${getEffects(builtin).size});
            |}
        """.trimMargin()
    }

    fun generateCtors(): String {
        var code = reactor.actions.joinToString(separator = "\n") { generateCtor(it) }
        if (hasStartup) code += generateCtor(BuiltinTrigger.STARTUP);
        if (hasShutdown) code += generateCtor(BuiltinTrigger.SHUTDOWN);
        return code;
    }

    fun generateSelfStruct(builtinTrigger: BuiltinTrigger) = with(PrependOperator) {
        """
            |typedef struct {
            |  ${if (builtinTrigger == BuiltinTrigger.STARTUP) "Startup" else "Shutdown"} super;
            |  ${if (getEffects(builtinTrigger).size > 0) "Reaction *_effects[${getEffects(builtinTrigger).size}];" else ""}
            |} ${builtinTrigger.codeType};   
            """.trimMargin()
    };

    fun generateSelfStructs(): String {
        var code = reactor.actions.joinToString(separator = "\n") { generateSelfStruct(it) }

        if (hasStartup) {
           code += generateSelfStruct(BuiltinTrigger.STARTUP) ;
        }
        if (hasShutdown) {
            code += generateSelfStruct(BuiltinTrigger.SHUTDOWN) ;
        }
        return code;
    }

    fun generateReactorStructFields(): String {
        var code = reactor.actions.joinToString(prefix = "// Actions and builtin triggers\n", separator = "\n", postfix = "\n") { "${it.codeType} ${it.name};" }
        if (hasStartup) code += "${BuiltinTrigger.STARTUP.codeType} startup;"
        if (hasShutdown) code += "${BuiltinTrigger.SHUTDOWN.codeType} shutdown;"
        return code;
    }

    fun generateReactorCtorCode(action: Action)  =  with(PrependOperator) {
        """
            |self->_triggers[trigger_idx++] = (Trigger *) &self->${action.name};
            |${action.codeType}_ctor(&self->${action.name}, &self->super, ${action.minDelay.toCCode()}, ${action.minSpacing.toCCode()});
            |
            """.trimMargin()
    };

    fun generateReactorCtorCode(builtin: BuiltinTrigger)  =  with(PrependOperator) {
        """
            |self->_triggers[trigger_idx++] = (Trigger *) &self->${builtin.codeName};
            |${builtin.codeType}_ctor(&self->${builtin.codeName}, &self->super);
            |
            """.trimMargin()
    };
    fun generateReactorCtorCodes(): String {
        var code = reactor.actions.joinToString(prefix = "// Initialize actions and builtin triggers\n", separator = "\n", postfix = "\n") { generateReactorCtorCode(it)}
        if(hasStartup) code += generateReactorCtorCode(BuiltinTrigger.STARTUP);
        if(hasShutdown) code += generateReactorCtorCode(BuiltinTrigger.SHUTDOWN);
        return code;
    }

}