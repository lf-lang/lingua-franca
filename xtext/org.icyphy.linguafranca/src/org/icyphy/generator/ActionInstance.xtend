package org.icyphy.generator

import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Variable

/** Instance of an action. */
class ActionInstance extends TriggerInstance<Variable> {
	
	new(Action definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create an ActionInstance with no parent.')
        }
	}
}