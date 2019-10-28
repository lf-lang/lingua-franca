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
import java.util.LinkedHashMap
import org.icyphy.linguaFranca.Reaction
import org.icyphy.generator.ReactionGraph.ReactionInstance

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
            this.inputs.add(new PortInstance(inputDecl, parent))
        }
        
        // Instantiate outputs for this reactor instance
        for (outputDecl : definition.reactorClass.outputs) {
            this.outputs.add(new PortInstance(outputDecl, parent))
        }
        
        // Populate destinations map.
        // Note that this can only happen _after_ the children and 
        // port instances have been created.
        for (connection : definition.reactorClass.connections) {
            var srcInstance = this.getPortInstance(connection.leftPort)
            var dstInstances = this.destinations.get(srcInstance)
            if (dstInstances === null) {
                dstInstances = new HashSet()
                this.destinations.put(srcInstance, dstInstances)   
            }
            dstInstances.add(this.getPortInstance(connection.rightPort))
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
    
    /** List of reaction instances for this reactor instance. */
    public var LinkedHashMap<Reaction, ReactionInstance> reactionInstances = new LinkedHashMap();
    
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

    /** Return the instance of a child rector created by the specified
     *  definition or null if there is none.
     *  @param definition The definition of the child reactor ("new" statement).
     *  @return The instance of the child reactor or null if there is no
     *   such "new" statement.
     */
    def getChildReactorInstance(Instantiation definition) {
        for (child : this.children) {
            if (child.definition === definition) {
                return child
            }
        }
        null
    }
     
    /** Given a port definition, return the port instance
     *  corresponding to that definition, or null if there is
     *  no such instance.
     *  @param port The port definition (a syntactic object in the AST).
     *  @return A port instance, or null if there is none.
     */
    def private lookupLocalPort(Port port) {
        // Search one of the inputs and outputs sets.
        var ports = null as HashSet<PortInstance>
        if (port instanceof Input) {
            ports = this.inputs
        } else if (port instanceof Output) {
            ports = this.outputs
        }
        for (portInstance : ports) {
            if (portInstance.definition === port) {
                return portInstance
            }
        }
        null
    }
     
    /** Given a reference to a port either belongs to this reactor
     *  instance or to a child reactor instance, return the port instance.
     *  Return null if there is no such instance.
     *  This is used for port references that have either the form of
     *  portName or reactorName.portName.
     *  @param reference The port reference.
     *  @return A port instance, or null if there is none.
     */
    def getPortInstance(VarRef reference) {
        if (!(reference.variable instanceof Port)) {
           // Trying to resolve something that is not a port
           return null
        }
        if (reference.container === null) {
            // Handle local reference
            return lookupLocalPort(reference.variable as Port)             
        } else {
             // Handle hierarchical reference
            var containerInstance = this.getChildReactorInstance(reference.container)
            return containerInstance.lookupLocalPort(reference.variable as Port) 
        }
    }
    
    /** Return the set of all ports that receive data from the 
     *  specified source. This includes inputs and outputs at the same level 
     *  of hierarchy and input ports deeper in the hierarchy.
     *  It does not include inputs or outputs up the hierarchy (i.e., ones
     *  that are reached via any output port that it does return).
     *  @param source An output or input port.
     */    
    def transitiveClosure(PortInstance source) {
        var result = new HashSet<PortInstance>();
        transitiveClosure(source, result);
        result
    }    
     
    /** Add to the destinations hash set all ports that receive data from the 
     *  specified source. This includes inputs and outputs at the same level 
     *  of hierarchy and input ports deeper in the hierarchy.
     *  It does not include inputs or outputs up the hierarchy (i.e., ones
     *  that are reached via any output port that it does return).
     *  
     *  @param destinations The set of destinations to populate.
     */    
    private def void transitiveClosure(PortInstance source, HashSet<PortInstance> destinations) {
        var localDestinations = this.destinations.get(source)
        
        for (destination : localDestinations?:emptyList) {
            destinations.add(destination)
            destination.parent.transitiveClosure(destination, destinations)
        }
    }
}