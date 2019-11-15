package org.icyphy.generator

import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Variable
import org.icyphy.linguaFranca.LinguaFrancaPackage

/** Instance of a timer. */
class TimerInstance extends TriggerInstance<Variable> {
	
	var startup = false
	
	new(Timer definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a TimerInstance with no parent.')
        }
        if (definition.name.equals(LinguaFrancaPackage.Literals.TRIGGER_REF__STARTUP.name)) {
        	this.startup = true
        }
	}
	
	def isStartup() {
    	this.startup
    }
}