package org.icyphy.generator

import org.eclipse.emf.ecore.EObject

abstract class NamedInstance<T extends EObject> {
	
	/** The Instantiation AST object from which this was created. */
    public var T definition
	
	public var ReactorInstance parent
	
	new(T definition, ReactorInstance parent) {
		this.definition = definition;
		this.parent = parent
	}
	
	/** Return the full name of this instance, which has the form
     *  "a.b.c", where "c" is the name of this instance, "b" is the name
     *  of its container, and "a" is the name of its container, stopping
     *  at the container in main.
     *  @return The full name of this instance.
     */
    def String getFullName() {
        var prefix = this.prefix
        if (prefix !== "") {
            this.prefix + '.' + this.getName
        } else {
            this.getName
        }
    }
    
     abstract def String getName();
	
	/** Return the full name of this instance's parent, which has the form
     *  "a.b", where "b" is the name of the parent and "a" is the name of
     *  its container.
     *  @return The full name of this instance.
     */
	protected def String getPrefix() {
    	if (parent !== null) {
            this.parent.getFullName
        } else {
            ""
        }
    }
	
	
}