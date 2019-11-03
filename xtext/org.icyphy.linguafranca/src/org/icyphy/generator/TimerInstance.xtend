package org.icyphy.generator

import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Variable

/** Instance of a timer. */
class TimerInstance extends TriggerInstance<Variable> {
	
	new(Timer definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a TimerInstance with no parent.')
        }
	}
}