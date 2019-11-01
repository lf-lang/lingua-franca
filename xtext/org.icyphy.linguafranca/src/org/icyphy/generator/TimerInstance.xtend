package org.icyphy.generator

import org.icyphy.linguaFranca.Timer

/** Instance of a timer. */
class TimerInstance extends NamedInstance<Timer> implements TriggerInstance {
	
	new(Timer definition, ReactorInstance parent) {
		super(definition, parent)
        if (parent === null) {
            throw new Exception('Cannot create a PortInstance with no parent.')
        }
	}
	
    /** Return the name of this timer. 
     *  @return The name of this timer.
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