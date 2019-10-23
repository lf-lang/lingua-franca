/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Port

/** An instance of a reactor or composite.
 */
class InstanceInfo {
	
	static val instanceToInfo = new HashMap<Instance, InstanceInfo>()
	
	static def get(Instance instance) {
		val info = instanceToInfo.get(instance)
		if (info === null) {
			return new InstanceInfo(instance)
		} else {
			return info
		}
	}
	
//	public static val childToParent = new HashMap<Instance, Instance>()
//	public static val parentToChildren = new HashMap<Instance, HashSet<Instance>>()
	
	private new(Instance instance) {
		this.instance = instance
		instanceToInfo.put(instance, this)
	}
	
//	new() {
//		
//	}
	
	/** Create an instance from the specified Instance object
	 *  and with the specified parent that instantiated it.
	 *  @param instance The Instance statement in the AST.
	 *  @param parent The parent.
	 */
	new(Instance instance, Instance parent) {
		this(instance)
		this.parent = parent
	}
	
	/** The contained instances, indexed by name. */
	public var HashSet<Instance> children = new HashSet<Instance>()
		
	/** The parent instance, or null if Main. */
	public var Instance parent
			
	/** The Instance AST object from which this was created. */
	public var Instance instance // FIXME: call this self?
		
	/** Port instances indexed by port. */
	public var portInstances = new HashMap<Port, ReactionGraph.PortInstance>()
	
	/** Properties associated with this instance.
	 *  This is used by particular code generators.
	 */
	public var HashMap<String,Object> properties = new HashMap<String,Object>()
	
	/////////////////////////////////////////////
	
//	/** Add a child instance.
//	 *  @param reactorInstance The contained instance.
//	 */
//	def addChild(Instance child) {
//		children.add(child)
//	}
	
//	/** Get a contained instance.
//	 *  @param instanceName The contained instance name.
//	 *  @return The contained instance, or null if there isn't one.
//	 */
//	def getContainedInstance(String name) {
//		return containedInstances.get(name)
//	}
	
//	/** Get children instances.
//	 *  @return The children instances
//	 */
//	def getContainedInstance(String name) {
//		return children
//	}
	
	/** Return the full name of this instance, which has the form
	 *  "a.b.c", where "c" is the name of this instance, "b" is the name
	 *  of its container, and "a" is the name of its container, stopping
	 *  at the container in main.
	 *  @return The full name of this instance.
	 */
	 def String getFullName() {
	 	if (parent !== null) {
	 		InstanceInfo.get(parent).getFullName() + '.' + instance.name
	 	} else {
	 		instance.name
	 	}
	 }
}