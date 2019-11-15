package org.icyphy.generator

import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Variable
import org.icyphy.linguaFranca.LinguaFrancaPackage

/** Instance of an action. */
class ActionInstance extends TriggerInstance<Variable> {
    
    var shutdown = false
    
    new(Action definition, ReactorInstance parent) {
        super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create an ActionInstance with no parent.')
        }
        if (definition.name.equals(LinguaFrancaPackage.Literals.TRIGGER_REF__SHUTDOWN.name)) {
        	this.shutdown = true
        }
    }
    
    def isShutdown() {
    	this.shutdown
    }
}