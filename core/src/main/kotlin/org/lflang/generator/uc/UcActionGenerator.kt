package org.lflang.generator.uc

import org.lflang.MessageReporter
import org.lflang.generator.PrependOperator
import org.lflang.generator.orZero
import org.lflang.inferredType
import org.lflang.isLogical
import org.lflang.lf.Action
import org.lflang.lf.BuiltinTrigger
import org.lflang.lf.Reactor

class UcActionGenerator(private val reactor: Reactor, private val messageReporter: MessageReporter) {
}
