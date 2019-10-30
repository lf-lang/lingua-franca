package org.icyphy.generator

import org.icyphy.linguaFranca.Action

class ActionInstance extends NamedInstance<Action> implements TriggerInstance {
	
	new(Action definition, ReactorInstance parent) {
		super(definition, parent)
	}
	
	override getName() {
		throw new UnsupportedOperationException("TODO: auto-generated method stub")
	}
	
}