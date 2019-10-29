/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashMap
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.VarRef
import org.eclipse.emf.common.util.EList

/** Representation of a runtime instance of a reactor.
 */
class ReactorInstance extends NamedInstance<Instantiation> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent.
     */
    new(Instantiation definition, ReactorInstance parent) {
        super(definition, parent)
        
        // Record how many times the definition has been used 
        // to create a new ReactorInstance
        var count = GeneratorBase.nameRegistry.get(this.prefix -> definition.name);
        if (count === null) {
        	count = 0
        }
        this.instantiationOrdinal = count++
        GeneratorBase.nameRegistry.put(this.prefix -> definition.name, this.instantiationOrdinal)
        
        // Record how many new ReactorInstance objects have been created
        count = ReactorInstance.instanceCounter.get(definition);
        if (count === null) {
        	count = 0
        }
        this.instanceOrdinal = count++
        ReactorInstance.instanceCounter.put(definition, this.instantiationOrdinal)
        
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
        
        // Create the reaction instances in this reactor instance
        // This also establishes all the implied dependencies
        createReactionInstances()
    }
    
    /** The contained instances, indexed by name. */
    public var HashSet<ReactorInstance> children = new HashSet<ReactorInstance>()

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
    public var LinkedHashMap<Reaction, ReactionInstance> reactionInstances = new LinkedHashMap(); // FIXME: Why is this not just an array?
    
    var instanceOrdinal = Integer.MIN_VALUE
    
    var instantiationOrdinal = Integer.MIN_VALUE
    
    static var HashMap<Instantiation, Integer> instanceCounter = new HashMap();
    
    /////////////////////////////////////////////
    
    
    def createReactionInstances() {
 		var reactions = this.definition.reactorClass.reactions
    	if (this.definition.reactorClass.reactions !== null) {
            var ReactionInstance previousReaction = null
            for (Reaction reaction : reactions) {
                // Create the reaction instance.
                var reactionInstance = new ReactionInstance(reaction, this)
                // If there is an earlier reaction in this same reactor, then
                // create a link
                // in the dependence graph.
                if (previousReaction !== null) {
                    previousReaction.dependentReactions.add(reactionInstance)
                    reactionInstance.dependsOnReactions.add(previousReaction)
                }
                previousReaction = reactionInstance;
                // Add the reaction instance to the map of reactions for this
                // reactor.
                this.reactionInstances.put(reactionInstance.definition,
                        reactionInstance);
                // nodes.add(reactionInstance); // FIXME

                // If the reaction is triggered by an input to this reactor
                // instance, then create a PortInstance for that port
                // (if it does not already exist)
                // and establish the dependency on that port.
                // Only consider inputs and outputs, ignore actions and timers.
                var EList<VarRef> deps = null;
                // First handle dependencies
                if (reaction.getTriggers() !== null) {
                    deps = reaction.getTriggers();
                }
                if (reaction.getSources() !== null) {
                    if (deps !== null) {
                        deps.addAll(reaction.getSources());
                    } else {
                        deps = reaction.getSources();
                    }
                }
                if (deps !== null) {
                    for (VarRef ref : deps) {
                        if (ref.getVariable() instanceof Input) {
                            var Input input = ref.getVariable as Input;
                            var PortInstance port = new PortInstance(
                                    input, this);
                            this.portInstances.put(input, port);
                            port.dependentReactions.add(reactionInstance);
                            reactionInstance.dependsOnPorts.add(port);
                        } else if (ref.variable instanceof Output) {
                            var ReactorInstance childInstance = this
                                    .getChildReactorInstance(
                                            ref.getContainer());
                                var output = ref.variable as Output;
                                var port = childInstance.portInstances
                                        .get(output);
                                if (port === null) {
                                    port = new PortInstance(output, childInstance);
                                    childInstance.portInstances.put(output, port);
                                }
                                port.dependentReactions.add(reactionInstance);
                                reactionInstance.dependentPorts.add(port);
                            
                        }
                    }
                }

                // Then handle anti-dependencies
                // If the reaction produces an output from this reactor
                // instance,
                // then create a PortInstance for that port (if it does not
                // already exist)
                // and establish the dependency on that port.
                if (reaction.effects !== null) {
                    for (VarRef antidep : reaction.getEffects()) {
                        // Check for dotted output, which is the input of a
                        // contained reactor.
                        if (antidep.variable instanceof Input) {
                            var childInstance = this
                                    .getChildReactorInstance(
                                            antidep.getContainer());
                                var input = antidep.getVariable() as Input;
                                var port = childInstance.portInstances
                                        .get(input);
                                if (port === null) {
                                    port = new PortInstance(input, childInstance);
                                    childInstance.portInstances.put(input, port);
                                }
                                port.dependsOnReactions.add(reactionInstance);
                                reactionInstance.dependentPorts.add(port);
                            
                        } else if (antidep.variable instanceof Output) {
                            var output = antidep.getVariable() as Output;
                            var port = this.portInstances
                                    .get(output);
                            if (port === null) {
                                port = new PortInstance(output, this);
                                this.portInstances.put(output, port);
                            }
                            port.dependsOnReactions.add(reactionInstance);
                            reactionInstance.dependentPorts.add(port);
                        }
                    }
                }
            }
        }
    	
    }
    
    
    /** Return the name of this instance. If other instances due to
     *  the same instantiation exist at the same level of hierarchy, 
     *  the name is appended with an additional index between braces 
     *  to disambiguate it from those other instances.
     *  @return The name of this instance.
     */
    override String getName() {
    	if (this.instantiationOrdinal > 0) {
    		this.definition.name + "(" + this.instantiationOrdinal + ")"
    	} else {
    		this.definition.name	
    	}
    }

	def String getInstanceID() {
		this.definition.name.toLowerCase + "_" + this.instanceOrdinal;
	}
	
	def String getInstantiationID() {
		this.definition.name.toLowerCase + "_" + this.instantiationOrdinal;
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