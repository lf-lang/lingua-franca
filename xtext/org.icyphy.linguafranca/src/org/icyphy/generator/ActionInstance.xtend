package org.icyphy.generator

import org.icyphy.linguaFranca.Action

/** Instance of an action. */
class ActionInstance extends NamedInstance<Action> implements TriggerInstance {
	
	new(Action definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create an ActionInstance with no parent.')
        }
	}
	
    /** Return the name of this action. 
     *  @return The name of this action.
     */
	override getName() {
		throw new UnsupportedOperationException("TODO: auto-generated method stub")
	}

    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        parent.main
    }	
}