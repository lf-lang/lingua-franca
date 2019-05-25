/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.LinkedHashMap
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.Instance

/** An instance of a reactor or composite.
 */
class ReactorInstance {
	
	/** Create new top-level instance. */
	new(Component component) {
		this.component = component
		this.instanceStatement = null
		this.container = null
	}
	
	/** Create an instance from the specified Instance statement
	 *  and with the specified container.
	 *  @param instance The Instance statement in the AST.
	 *  @param container The container.
	 */
	new(Component component, Instance instanceStatement, ReactorInstance container) {
		this.component = component
		this.instanceStatement = instanceStatement
		this.container = container
	}
	
	/** The contained instances, indexed by name. */
	public var LinkedHashMap<String,ReactorInstance> containedInstances
			= new LinkedHashMap<String,ReactorInstance>()
	
	/** The AST Component object defining this instance.
	 *  This can be null if the component is defined outside Lingua Franca.
	 */
	public var Component component
	
	/** The container instance, or null if Main. */
	public var ReactorInstance container
	
	/** The Instance AST object from which this was created. */
	public var Instance instanceStatement
	
	/** Properties associated with this instance.
	 *  This is used by particular code generators.
	 */
	public var HashMap<String,Object> properties = new HashMap<String,Object>()
	
	/////////////////////////////////////////////
	
	/** Add a contained instance.
	 *  @param reactorInstance The contained instance.
	 */
	def addContainedInstance(ReactorInstance reactorInstance) {
		// This assumes that the name is unique, something checked by the validator.
		containedInstances.put(reactorInstance.instanceStatement.name, reactorInstance)
	}
	
	/** Get a contained instance.
	 *  @param instanceName The contained instance name.
	 *  @return The contained instance, or null if there isn't one.
	 */
	def getContainedInstance(String name) {
		return containedInstances.get(name)
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
	 
	 /** Return the name of this instance within its container. */
	 def String getName() {
	 	instanceStatement.name
	 }
}