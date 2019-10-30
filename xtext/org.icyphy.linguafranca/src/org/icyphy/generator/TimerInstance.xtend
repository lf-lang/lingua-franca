package org.icyphy.generator

import org.icyphy.linguaFranca.Timer

class TimerInstance extends NamedInstance<Timer> implements TriggerInstance {
	
	new(Timer definition, ReactorInstance parent) {
		super(definition, parent)
	}
	
	override getName() {
		throw new UnsupportedOperationException("TODO: auto-generated method stub")
	}
	
}