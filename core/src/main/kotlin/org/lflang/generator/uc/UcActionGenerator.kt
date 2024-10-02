package org.lflang.generator.uc

import org.lflang.MessageReporter
import org.lflang.generator.PrependOperator
import org.lflang.generator.orZero
import org.lflang.inferredType
import org.lflang.isLogical
import org.lflang.lf.*
import org.lflang.priority

class UcActionGenerator(private val reactor: Reactor, private val messageReporter: MessageReporter) {
    companion object { /** Get the "name" a reaction is represented with in target code.*/
    val Action.codeType
        get(): String = "Action_$name"

    val BuiltinTriggerRef.codeType
        get(): String =
            if (type== BuiltinTrigger.STARTUP) {
                "Startup_$name"
            } else if (type == BuiltinTrigger.SHUTDOWN) {
                "Shutdown_$name"
            } else {
                ""
            }
    }
}