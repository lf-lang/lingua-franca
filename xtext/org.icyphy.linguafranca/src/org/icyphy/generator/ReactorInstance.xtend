/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import org.icyphy.generator.PortInstance
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.VarRef

/** Representation of a runtime instance of a reactor.
 */
class ReactorInstance {
		
	/** Create a runtime instance from the specified definition
	 *  and with the specified parent that instantiated it.
	 *  @param instance The Instance statement in the AST.
	 *  @param parent The parent.
	 */
	new(Instantiation definition, ReactorInstance parent) {
		this.definition = definition
		this.parent = parent
		
		// Instantiate children for this reactor instance
		for (child : definition.reactorClass.instantiations) {
			var childInstance = new ReactorInstance(child, this)
			this.children.add(childInstance)
		}
		
		// Instantiate inputs for this reactor instance
		for (inputDecl : definition.reactorClass.inputs) {
			this.inputs.add(new PortInstance(parent, inputDecl))
		}
		
		// Instantiate outputs for this reactor instance
		for (outputDecl : definition.reactorClass.outputs) {
			this.outputs.add(new PortInstance(parent, outputDecl))
		}
		
		// Populate destinations map.
		// Note that this can only happen _after_ the children and 
		// port instances have been created.
		for (connection : definition.reactorClass.connections) {
			var srcInstance = this.getPortInstance(connection.leftPort)
			var destinations = this.destinations.get(srcInstance)
			if (destinations === null) {
				destinations = new HashMap()
				this.destinations.put(srcInstance, destinations)	
			}
			destinations.add(this.getPortInstance(connection.rightPort))
		}
	}
	
	/** The contained instances, indexed by name. */
	public var HashSet<ReactorInstance> children = new HashSet<ReactorInstance>()

	/** The Instantiation AST object from which this was created. */
	public var Instantiation definition

	/** A map from sources to destinations as specified by the connections of this reactor instance. */
	public var HashMap<PortInstance, HashSet<PortInstance>> destinations = new HashMap();

	/** The input port instances belonging to this reactor instance. */	
	public var inputs = new HashSet<PortInstance>	

	/** The output port instances belonging to this reactor instance. */	
	public var outputs = new HashSet<PortInstance>	
		
	/** The reactor instance that instantiated this reactor instance, or null if Main. */
	public var ReactorInstance parent
			
	/** Port instances indexed by port. */
	public var portInstances = new HashMap<Port, PortInstance>()
	
	/** Properties associated with this instance.
	 *  This is used by particular code generators.
	 */
	public var HashMap<String,Object> properties = new HashMap<String,Object>()
	
	/////////////////////////////////////////////
	
	/** Return the full name of this instance, which has the form
	 *  "a.b.c", where "c" is the name of this instance, "b" is the name
	 *  of its container, and "a" is the name of its container, stopping
	 *  at the container in main.
	 *  @return The full name of this instance.
	 */
	 def String getFullName() {
	 	if (parent !== null) {
	 		this.parent.getFullName() + '.' + definition.name
	 	} else {
	 		definition.name
	 	}
	 }
	 
	 def getChildReactorInstance(Instantiation definition) {
	 	for (child : this.children) {
			if (child.definition === definition) {
				return child
			}
		}
	 }
	 
	 def private lookupLocalPort(Port port) {

		var ports = null as HashSet<PortInstance>

		if (port instanceof Input) {
			ports = this.inputs
		} else if (port instanceof Output) {
			ports = this.outputs
		}
		for (portInstance : ports) {
			if (portInstance.portDecl === port) {
				return portInstance
			}
		}

	}
	 
	// FIXME: docs	 
	 def getPortInstance(VarRef ref) {
	 	
	 	if (!(ref.variable instanceof Port)) {
			// Trying to resolve something that is not a port
			return null
		}
	 	
	 	if (ref.container === null) {
	 		// Handle local reference
	 		return lookupLocalPort(ref.variable as Port) 			
	 	} else {
	 		// Handle hierarchical reference
			var containerInstance = this.getChildReactorInstance(ref.container)
			return containerInstance.lookupLocalPort(ref.variable as Port) 
	 	}
	 	
	 }
	 
	 
	 
	 /** Add to the destinations hash set all ports that receive data from the 
	 *  specified source. This includes inputs and outputs at the same level 
	 *  of hierarchy, input ports deeper in the hierarchy, and output up the
	 *  hierarchy. // FIXME: do we really want to record all of these, or only the inputs because only they can trigger?
	 */	
	def void transitiveClosure(PortInstance source, HashSet<PortInstance> destinations) {
		//var localDestinations = ReactorInfo.get(this.definition.reactorClass).sourceToDestinations.get(source);
		var localDestinations = this.destinations.get(source)
		
		for (destination : localDestinations?:emptyList) {
			destinations.add(destination)
			destination.parent.transitiveClosure(destination, destinations)
			//if (destination.container === null) {
//				if (destination.variable instanceof Output && this.parent !== null) {
//					// Search one level higher // NOTE: moved this from ReactorInfo in order to get access to this.parent
//					this.parent.transitiveClosure(destination.variable as Output, destinations)
//				} else {
					// Search on same level
				//	this.transitiveClosure(destination.variable as Port, destinations)
//				//}
//			} else if (destination.variable instanceof Input) {
//				// Search one level deeper
//				destination.container
//				//.transitiveClosure(destination.variable as Input, destinations)
//			}
		}
	}
}