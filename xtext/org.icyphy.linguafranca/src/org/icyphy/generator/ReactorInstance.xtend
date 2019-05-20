package org.icyphy.generator

import java.util.HashMap
import java.util.LinkedHashMap
import org.icyphy.linguaFranca.Instance

/** An instance of a reactor or composite.
 */
class ReactorInstance {
	
	/** Create new top-level instance. */
	new() {
		this.instanceStatement = null
		this.container = null
	}
	
	/** Create an instance from the specified Instance statement
	 *  and with the specified container.
	 *  @param instance The Instance statement in the AST.
	 *  @param container The container.
	 */
	new(Instance instanceStatement, ReactorInstance container) {
		this.instanceStatement = instanceStatement
		this.container = container
	}
	
	/** The contained instances, indexed by name. */
	var LinkedHashMap<String,ReactorInstance> containedInstances
			= new LinkedHashMap<String,ReactorInstance>()
			
	/** The container instance, or null if Main. */
	var ReactorInstance container
	
	/** The Instance AST object from which this was created. */
	public var Instance instanceStatement
	
	/** Properties associated with this instance.
	 *  This is used by particular code generators.
	 */
	public var HashMap<String,String> properties = new HashMap<String,String>()
	
	/////////////////////////////////////////////
	
	/** Add a contained instance.
	 *  @param reactorInstance The contained instance.
	 */
	def addContainedInstance(ReactorInstance reactorInstance) {
		// This assumes that the name is unique, something checked by the validator.
		containedInstances.put(reactorInstance.instanceStatement.name, reactorInstance)
	}
	
	/** Return the full name of this instance, which has the form
	 *  "a.b.c", where "c" is the name of this instance, "b" is the name
	 *  of its container, and "a" is the name of its container, stopping
	 *  at the container in main.
	 *  @return The full name of this instance.
	 */
	 def String getFullName() {
	 	if (container !== null) {
	 		container.getFullName() + '.' + instanceStatement.name
	 	} else {
	 		instanceStatement.name
	 	}
	 }
}