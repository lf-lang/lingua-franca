/** A data structure for a reactor instance. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator

import java.util.ArrayList
import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.List
import org.lflang.ASTUtils
import org.lflang.lf.Action
import org.lflang.lf.Connection
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Output
import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Timer
import org.lflang.lf.Value
import org.lflang.lf.VarRef
import org.lflang.lf.Variable
import org.lflang.lf.WidthSpec

import static extension org.lflang.ASTUtils.*

/**
 * Representation of a runtime instance of a reactor.
 * For the main reactor, which has no parent, once constructed,
 * this object represents the entire Lingua Franca program.
 * The constructor analyzes the graph of dependencies between
 * reactions and throws exception if this graph is cyclic.
 *
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ReactorInstance extends NamedInstance<Instantiation> {

    /**
     * A representation of the dependencies between reactions as they are
     * inferred from the main reactor instance.
     */
    public static var ReactionInstanceGraph reactionGraph

    /**
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param instance The Instance statement in the AST.
     * @param parent The parent, or null for the main rector.
     * @param generator The generator (for error reporting).
     */
    new(Instantiation definition, ReactorInstance parent, GeneratorBase generator) {
        // If the reactor is being instantiated with new[width], then pass -2
        // to the constructor, otherwise pass -1.
        this(definition, parent, generator, (definition.widthSpec !== null)? -2 : -1)
    }

    /**
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param instance The Instance statement in the AST.
     * @param parent The parent, or null for the main rector.
     * @param generator The generator (for error reporting).
     * @param reactorIndex -1 for an ordinary reactor, -2 for a
     * placeholder for a bank of reactors, or the index of the
     * reactor in a bank of reactors otherwise.
     */
    protected new(Instantiation definition, ReactorInstance parent,
        GeneratorBase generator, int reactorIndex) {
        super(definition, parent)
        this.generator = generator
        this.bankIndex = reactorIndex
        
        // If this reactor is actually a bank of reactors, then instantiate
        // each individual reactor in the bank and skip the rest of the
        // initialization for this reactor instance.
        if (reactorIndex === -2) {
            // If the bank width is variable, then we have to wait until the first connection
            // before instantiating the children.
            var width = width(definition.widthSpec)
            if (width > 0) {
                this.bankMembers = new ArrayList<ReactorInstance>(width)
                for (var index = 0; index < width; index++) {
                    var childInstance = new ReactorInstance(definition, parent, generator, index)
                    this.bankMembers.add(childInstance)
                    childInstance.bank = this
                    childInstance.bankIndex = index
                }
            } else {
                generator.reportError(definition, "Cannot infer width.")
            }
            return
        }
        
        // Apply overrides and instantiate parameters for this reactor instance.
        for (parameter : definition.reactorClass.toDefinition.allParameters) {
            this.parameters.add(new ParameterInstance(parameter, this))
        }

        // Instantiate children for this reactor instance
        for (child : definition.reactorClass.toDefinition.allInstantiations) {
            var childInstance = new ReactorInstance(child, this, generator)
            this.children.add(childInstance)
            // If the child is a bank of instances, add all the bank instances.
            // These must be added after the bank itself.
            if (childInstance.bankMembers !== null) {
                this.children.addAll(childInstance.bankMembers)
            }
        }

        // Instantiate inputs for this reactor instance
        for (inputDecl : definition.reactorClass.toDefinition.allInputs) {
            if (inputDecl.widthSpec === null) {
                this.inputs.add(new PortInstance(inputDecl, this))
            } else {
                this.inputs.add(new MultiportInstance(inputDecl, this, generator))
            }
        }

        // Instantiate outputs for this reactor instance
        for (outputDecl : definition.reactorClass.toDefinition.allOutputs) {
            if (outputDecl.widthSpec === null) {
                this.outputs.add(new PortInstance(outputDecl, this))
            } else {
                this.outputs.add(new MultiportInstance(outputDecl, this, generator))
            }
        }

        // Instantiate timers for this reactor instance
        for (timerDecl : definition.reactorClass.toDefinition.allTimers) {
            this.timers.add(new TimerInstance(timerDecl, this))
        }

        // Instantiate actions for this reactor instance
        for (actionDecl : definition.reactorClass.toDefinition.allActions) {
            this.actions.add(new ActionInstance(actionDecl, this))
        }

        // Populate destinations map and the connectivity information
        // in the port instances.
        // Note that this can only happen _after_ the children and 
        // port instances have been created.
        
        // Unfortunately, we have to do some complicated things here
        // to support multiport-to-multiport, multiport-to-bank,
        // and bank-to-multiport communication.  The principle being followed is:
        // In each connection statement, for each port instance on the left,
        // as obtained by the nextPort() function, connect to the next available
        // port on the right, as obtained by the nextPort() function.
        for (connection : definition.reactorClass.toDefinition.allConnections) {
            var leftPort = connection.leftPorts.get(0)
            var leftPortCount = 1
            for (rightPort : connection.rightPorts) {
                var rightPortInstance = nextPort(rightPort)
                while (rightPortInstance !== null) {
                    var leftPortInstance = nextPort(leftPort)
                    if (leftPortInstance === null) {
                        // We have run out of left ports. We may have also run out of
                        // right ports, so get a new left port only if there is one.
                        // We do not rely on the validator to ensure that the connection is balanced.
                        if (leftPortCount < connection.leftPorts.length) {
                            leftPort = connection.leftPorts.get(leftPortCount++)
                            leftPortInstance = nextPort(leftPort)
                        } else {
                            // If left ports are to be iterated, then start over here.
                            if (connection.isIterated) {
                                leftPort = connection.leftPorts.get(0)
                                // The left port may be in a bank.
                                if (leftPort.container !== null) {
                                    val reactor = getChildReactorInstance(leftPort.container)
                                    if (reactor !== null && reactor.bankMembers !== null) {
                                        for (bankReactor : reactor.bankMembers) {
                                            // Also reset the bank index.
                                            nextBankTable.put(leftPort, 0)
                                            // Since we are starting over with the bank, need to
                                            // remove all portInstances in each bank reactor that
                                            // match the left port.
                                            var portInstance = bankReactor.lookupPortInstance(leftPort.variable as Port)
                                            nextPortTable.remove(portInstance)
                                        }
                                    }
                                }
                                leftPortCount = 1
                                leftPortInstance = nextPort(leftPort)
                            } else {
                                leftPortInstance = null
                                generator.reportWarning(rightPort, "Unconnected ports on the right.")
                            }
                        }
                    }
                    // Do not make a connection if there is no new left port.
                    if (leftPortInstance !== null) {
                        connectPortInstances(connection, leftPortInstance, rightPortInstance)
                    }
                    rightPortInstance = nextPort(rightPort)
                }
                // At this point, rightPortInstance is null.
                // Go to the next right port if there is one.
            }
            // We are out of right ports.
            // Make sure we are out of left ports also.
            if (nextPort(leftPort) !== null || leftPortCount < connection.leftPorts.length - 1) {
                generator.reportWarning(leftPort, "Source is wider than the destination. Outputs will be lost.")
            }
        }
        
        // Check for dangling inputs or outputs and issue a warning.
        checkForDanglingConnections()

        // Create the reaction instances in this reactor instance.
        // This also establishes all the implied dependencies.
        // Note that this can only happen _after_ the children, 
        // port, action, and timer instances have been created.
        createReactionInstances()

        // If this is the main reactor, then perform static analysis.
        if (parent === null) {
            ReactorInstance.reactionGraph = new ReactionInstanceGraph(this)
        }
    }
        
    /** Data structure used by nextPort() to keep track of the next available bank. */
    var nextBankTable = new LinkedHashMap<VarRef,Integer>()

    /** Data structure used by nextPort() to keep track of the next available port. */
    var nextPortTable = new LinkedHashMap<PortInstance,Integer>()
    
    /**
     * Check for dangling connections.
     */
    def checkForDanglingConnections() {
        // FIXME identifies only dangling inputs
        // First, check that each bank index is either 0 (allowed for sources)
        // or equals the width of the bank, meaning all banks were used.
        for (portReference : nextBankTable.keySet) {
            var nextBank = nextBankTable.get(portReference)
            var reactor = this
            if (portReference.container !== null) {
                reactor = getChildReactorInstance(portReference.container)
            }
            // The reactor should be a bank.
            if (nextBank != 0 && nextBank < reactor.bankMembers.size) {
                // Not all the bank members were used.
                generator.reportWarning(portReference, "Not all bank members are connected.")
            }
        }
        // Next, check multiports.
        for (portInstance : nextPortTable.keySet) {
            var nextPort = nextPortTable.get(portInstance)
            // This port may or may not be a multiport.
            if (portInstance instanceof MultiportInstance) {
                if (nextPort != 0 && nextPort < portInstance.width) {
                    // Not all the ports were used.
                    generator.reportWarning(portInstance.parent.definition,
                            "Not all multiport input channels are connected.")
                }
            }
        }
    }
    
    /**
     * Given a VarRef for either the left or the right side of a connection
     * statement within this reactor instance, return the next available port instance
     * for that connection. If there are no more available ports, return null.
     * 
     * This handles parallel connections like `a.out, b.out -> c.in, d.in`.
     * Each VarRef on each side may also refer to a port of this reactor, as in
     * `in1, in2 -> b.in`.
     * To realize such connection statements, make connections until one side or
     * the other of the connection statement causes this method to return null,
     * then proceed to the next port on that side.
     * 
     * If the reactor is not a bank of reactors and the
     * port is not a multiport, then the returned port instance will simply
     * be the port referenced, which may be a port of this reactor or a port
     * of a contained reactor, unless that same port was returned by the previous
     * invocation of this method, in which case the returned value will be null.
     * 
     * If the port is a multiport, then this method will iterate through the ports
     * in the multiport, returning a new port each time it is called, until all ports
     * in the multiport have been returned. Then it will return null.
     * 
     * If the reactor is a bank of reactors, then this method will iterate through
     * the bank, returning a new port each time it is called, until all ports in all
     * banks are exhausted, at which time it will return null.
     * 
     * In all cases, upon returning null, if the port is source of data (an input port of
     * this reactor or an output port of a contained reactor), then this will reset the
     * state so that on the next call the first port will again be returned.
     * This allows source ports to be reused, multicasting their outputs.
     * If the port is not a source of data, no reset occurs because it is not
     * legal to make multiple connections to a sink port, whereas it is legal for
     * a source connection. All subsequent calls will return null.
     * 
     * @param portReference The port reference in the connection.
     */
    def PortInstance nextPort(VarRef portReference) {
        // First, figure out which reactor we are dealing with.
        // The reactor we want is the container of the port.
        // It may be a bank, in which case we want next available
        // bank member for the specified port.
        // If the port reference has no container, then the reactor is this one,
        // or if this one is a bank, the next available bank member.
        var reactor = this
        if (portReference.container !== null) {
            reactor = getChildReactorInstance(portReference.container)
        }
        // The reactor may be a bank, in which case, we will also need
        // the member reactor within the bank.
        var memberReactor = reactor
        var bankIdx = -1 // Indicator that this reactor is not a bank.
        if (reactor.bankMembers !== null) {
            // It is a bank.
            bankIdx = nextBankTable.get(portReference)?:0
            // If the bank is exhausted, return null.
            if (bankIdx >= reactor.bankMembers.size) {
                // If it is a source, reset the bank counter.
                if (portReference.isSource) {
                    nextBankTable.put(portReference, 0)
                }
                return null
            }
            memberReactor = reactor.bankMembers.get(bankIdx)
        }
        
        var portInstance = memberReactor.lookupPortInstance(portReference.variable as Port)
        
        if (portInstance === null) {
            generator.reportError(portReference, "No such port.")
            return null
        }
        
        // If the port is a multiport, then retrieve the next available port.
        if (portInstance instanceof MultiportInstance) {
            // Do not allow width 0 multiports.
            if (portInstance.width === 0) {
                generator.reportError(portReference, "Multiport of width zero is not allowed")
                return null
            }
            var portIndex = nextPortTable.get(portInstance)?:0

            if (portIndex >= portInstance.width) {
                // Multiport is exhausted. Will return null unless it's in a bank.
                // If this port is a source, reset port index to allow multicast.
                if (portReference.isSource) {
                    nextPortTable.put(portInstance, 0)
                }
                // If this is a bank, move to the next bank element.
                if (reactor.bankMembers !== null) {
                    bankIdx++
                    nextBankTable.put(portReference, bankIdx)
                    if (bankIdx >= reactor.bankMembers.size) {
                        // Bank is exhausted as well. Will return null.
                        // If this port is a source, reset the bank index.
                        if (portReference.isSource) {
                            nextBankTable.put(portReference, 0)
                        }
                        return null
                    }
                    // Get the bank member reactor.
                    memberReactor = reactor.bankMembers.get(bankIdx)
                    // Have a new port instance.
                    portInstance = memberReactor.lookupPortInstance(portReference.variable as Port)
                    // This should also be a multiport.
                    nextPortTable.put(portInstance, 1)
                    return (portInstance as MultiportInstance).getInstance(0)
                } else {
                    // Not a bank. Will return null.
                    return null
                }
            } else {
                // Multiport is not exhausted.
                nextPortTable.put(portInstance, portIndex + 1)
                return (portInstance as MultiportInstance).getInstance(portIndex)
            }
        } else {
            // The port is not a multiport.
            // If it is a bank, increment the bank counter.
            if (reactor.bankMembers !== null) {
                nextBankTable.put(portReference, bankIdx + 1)
            }
            var portIndex = nextPortTable.get(portInstance)?:0
            if (portIndex > 0) {
                // Port has been used.
                // If it is a source, reset the nextPortTable so it can be reused.
                if (portReference.isSource) {
                    nextPortTable.put(portInstance, 0)
                }
                return null
            }
            nextPortTable.put(portInstance, 1)
            return portInstance
        }
    }
    
    /**
     * Return true if the specified variable reference is a source of data.
     * It is a source of data if it is an output port of a contained reactor
     * or an input port of the current reactor.
     */
    def isSource(VarRef portReference) {
        (portReference.variable instanceof Output && portReference.container !== null)
        || (portReference.variable instanceof Input && portReference.container === null)
    }
    
    /**
     * Connect the given left port instance to the given right port instance.
     * These are both assumed to be single ports, not multiports.
     * @param connection The connection statement creating this connection.
     * @param srcInstance The source instance (the left port).
     * @param dstInstance The destination instance (the right port).
     */
    def connectPortInstances(Connection connection, PortInstance srcInstance, PortInstance dstInstance) {
        srcInstance.dependentPorts.add(dstInstance)
        if (dstInstance.dependsOnPort !== null && dstInstance.dependsOnPort !== srcInstance) {
            generator.reportError(
                connection,
                "dstInstance port " + dstInstance.getFullName + " is already connected to " +
                    dstInstance.dependsOnPort.getFullName
            )
            return
        }
        dstInstance.dependsOnPort = srcInstance
        var dstInstances = this.destinations.get(srcInstance)
        if (dstInstances === null) {
            dstInstances = new LinkedHashSet<PortInstance>()
            this.destinations.put(srcInstance, dstInstances)
        }
        dstInstances.add(dstInstance)
    }
    
    /** 
     * Override the base class to return the uniqueID of the bank rather
     * than this member of the bank, if this is a member of a bank of reactors.
     * 
     * @return An identifier for this instance that is guaranteed to be
     *  unique within the top-level parent.
     */
    override uniqueID() {
        if (this.bank !== null) {
            return this.bank.uniqueID
        }
        return super.uniqueID
    }

    // ////////////////////////////////////////////////////
    // // Public fields.
    
    /** The action instances belonging to this reactor instance. */
    public var actions = new LinkedList<ActionInstance>
    
    /** 
     * The contained reactor instances, in order of declaration.
     * For banks of reactors, this includes both the bank definition
     * Reactor (which has bankIndex == -2) followed by each of the
     * bank members (which have bankIndex >= 0).
     */
    public var LinkedList<ReactorInstance> children = new LinkedList<ReactorInstance>()

    /** A map from sources to destinations as specified by the connections
     *  of this reactor instance. Note that this is redundant, because the same
     *  information is available in the port instances of this reactor and its
     *  children, but it is sometimes convenient to have it collected here.
     */
    public var LinkedHashMap<PortInstance, LinkedHashSet<PortInstance>> destinations = new LinkedHashMap();

    /** The input port instances belonging to this reactor instance. */
    public var inputs = new LinkedList<PortInstance>

    /** The output port instances belonging to this reactor instance. */
    public var outputs = new LinkedList<PortInstance>

    /** The parameters of this instance. */
    public var parameters = new LinkedList<ParameterInstance>

    /** List of reaction instances for this reactor instance. */
    public var reactions = new LinkedList<ReactionInstance>();

    /** If non-null, then this reactor has a shutdown action that
     *  needs to be scheduled prior to shutting down the program.
     */
    public var ActionInstance shutdownActionInstance = null

    /** The timer instances belonging to this reactor instance. */
    public var timers = new LinkedList<TimerInstance>

    // ////////////////////////////////////////////////////
    // // Public methods.
    
    /** 
     * Override the base class to append [index] if this reactpr
     * is in a bank of reactors.
     * 
     * @return The full name of this instance.
     */
    override String getName() {
        var result = this.definition.name
        if (this.bankIndex >= 0) {
            result += "[" + this.bankIndex + "]"
        }
        result
    }

    /** 
     * Return the shutdown action within this reactor instance.
     * 
     * @return The corresponding shutdown action instance or null
     *  if this reactor instance does not have a shutdown action.
     */
    def getShutdownAction() {
        for (actionInstance : actions) {
            if (actionInstance.isShutdown) {
                return actionInstance
            }
        }
    }

    /** 
     * Return the startup timer within this reactor instance.
     * 
     * @return The corresponding startup timer instance or null
     *  if this reactor instance does not have a shutdown timer.
     */
    def getStartupTimer() {
        for (timerInstance : timers) {
            if (timerInstance.isStartup) {
                return timerInstance
            }
        }
    }

    /** 
     * Return the instance of a child rector created by the specified
     * definition or null if there is none.
     * 
     * @param definition The definition of the child reactor ("new" statement).
     * 
     * @return The instance of the child reactor or null if there is no
     *  such "new" statement.
     */
    def getChildReactorInstance(Instantiation definition) {
        for (child : this.children) {
            if (child.definition === definition) {
                return child
            }
        }
        null
    }

    /** 
     * Return the trigger instances (input ports, timers, and actions
     * that trigger reactions) belonging to this reactor instance.
     * 
     * @return The trigger instances belonging to this reactor instance.
     */
    def getTriggers() {
        var triggers = new LinkedHashSet<TriggerInstance<Variable>>
        for (reaction : this.reactions) {
            triggers.addAll(reaction.triggers)
        }
        return triggers
    }

    /** 
     * Return the trigger instances (input ports, timers, and actions
     * that trigger reactions) together the ports that the reaction reads
     * but that don't trigger it.
     * 
     * @return The trigger instances belonging to this reactor instance.
     */
    def getTriggersAndReads() {
        var triggers = new LinkedHashSet<TriggerInstance<Variable>>
        for (reaction : this.reactions) {
            triggers.addAll(reaction.triggers)
            triggers.addAll(reaction.reads)
        }
        return triggers
    }
        
    /**
     * Given a parameter definition for this reactor, return the initial value
     * of the parameter. If the parameter is overridden when instantiating
     * this reactor or any of its containing reactors, use that value.
     * Otherwise, use the default value in the reactor definition.
     * 
     * The returned list of Value objects is such that each element is an
     * instance of Time, String, or Code, never Parameter.
     * For most uses, this list has only one element, but parameter
     * values can be lists of elements, so the returned value is a list.
     * 
     * @param parameter The parameter definition (a syntactic object in the AST).
     * 
     * @return A list of Value objects, or null if the parameter is not found.
     *  Return an empty list if no initial value is given.
     *  Each value is an instance of Literal if a literal value is given,
     *  a Time if a time value was given, or a Code, if a code value was
     *  given (text in the target language delimited by {= ... =}
     */
    def List<Value> initialParameterValue(Parameter parameter) {
        return ASTUtils.initialValue(parameter, instantiations());
    }

    /**
     * Given a parameter definition for this reactor, return the initial integer
     * value of the parameter. If the parameter is overridden when instantiating
     * this reactor or any of its containing reactors, use that value.
     * Otherwise, use the default value in the reactor definition.
     * If the parameter cannot be found or its value is not an integer, return null.
     * 
     * @param parameter The parameter definition (a syntactic object in the AST).
     * 
     * @return An integer value or null.
     */
    def Integer initialIntParameterValue(Parameter parameter) {
        return ASTUtils.initialValueInt(parameter, instantiations());
    }

    /**
     * Return a list of Instantiation objects such that the first object
     * is the AST instantiation that created this reactor instance, the
     * second is the AST instantiation that created the containing
     * reactor instance, and so on until there are no more containing
     * reactor instances. This will return an empty list if this
     * reactor instance is at the top level (is main).
     */
    def List<Instantiation> instantiations() {
        if (_instantiations === null) {
            _instantiations = new LinkedList<Instantiation>();
            if (definition !== null) {
                _instantiations.add(definition);
                if (parent !== null) {
                    _instantiations.addAll(parent.instantiations());
                }
            }
        }
        return _instantiations;
    }

    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        if (this.parent === null) {
            this
        } else {
            parent.main
        }
    }
    
    /** Return a descriptive string. */
    override toString() {
        "ReactorInstance " + getFullName
    }

    /** 
     * Return the set of all ports that receive data from the 
     * specified source. This includes inputs and outputs at the same level 
     * of hierarchy and input ports deeper in the hierarchy.
     * It does not include inputs or outputs up the hierarchy (i.e., ones
     * that are reached via any output port that it does return).
     * If the argument is an input port, then it is included in the result.
     * No port will appear more than once in the result.
     * 
     * @param source An output or input port.
     */
    def transitiveClosure(PortInstance source) {
        var result = new LinkedHashSet<PortInstance>();
        transitiveClosure(source, result);
        result
    }
    
    ///////////////////////////////////////////////////
    //// Methods for finding instances in this reactor given an AST node.
    
    /** Return the action instance within this reactor 
     *  instance corresponding to the specified action reference.
     *  @param action The action as an AST node.
     *  @return The corresponding action instance or null if the
     *   action does not belong to this reactor.
     */
    def ActionInstance lookupActionInstance(Action action) {
        for (actionInstance : actions) {
            if (actionInstance.name.equals(action.name)) {
                return actionInstance
            }
        }
    }

    /** 
     * Given a parameter definition, return the parameter instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param parameter The parameter definition (a syntactic object in the AST).
     * @return A parameter instance, or null if there is none.
     */
    def ParameterInstance lookupParameterInstance(Parameter parameter) {
        return this.parameters.findFirst [
            it.definition === parameter
        ]
    }
    
    /** 
     * Given a port definition, return the port instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param port The port definition (a syntactic object in the AST).
     * @return A port instance, or null if there is none.
     */
    def PortInstance lookupPortInstance(Port port) {
        // Search one of the inputs and outputs sets.
        var LinkedList<PortInstance> ports = null
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

    /** Given a reference to a port belonging to this reactor
     *  instance, return the port instance.
     *  Return null if there is no such instance.
     *  @param reference The port reference.
     *  @return A port instance, or null if there is none.
     */
    def lookupPortInstance(VarRef reference) {
        if (!(reference.variable instanceof Port)) {
            // Trying to resolve something that is not a port
            return null
        }
        if (reference.container === null) {
            // Handle local reference
            return lookupPortInstance(reference.variable as Port)
        } else {
            // Handle hierarchical reference
            var containerInstance = this.
                getChildReactorInstance(reference.container)
            if (containerInstance === null) return null
            return containerInstance.lookupPortInstance(reference.variable as Port)
        }
    }

    /** Return the reaction instance within this reactor 
     *  instance corresponding to the specified reaction.
     *  @param reaction The reaction as an AST node.
     *  @return The corresponding reaction instance or null if the
     *   reaction does not belong to this reactor.
     */
    def lookupReactionInstance(Reaction reaction) {
        for (reactionInstance : reactions) {
            if (reactionInstance.definition === reaction) {
                return reactionInstance
            }
        }
    }
    
    /**
     * Return the reactor instance within this reactor
     * that has the specified instantiation. Note that this
     * may be a bank of reactors.
     */
    def lookupReactorInstance(Instantiation instantiation) {
        for (reactorInstance : children) {
            if (reactorInstance.definition === instantiation) {
                return reactorInstance
            }
        }
    }
    
    /** Return the timer instance within this reactor 
     *  instance corresponding to the specified timer reference.
     *  @param timer The timer as an AST node.
     *  @return The corresponding timer instance or null if the
     *   timer does not belong to this reactor.
     */
    def lookupTimerInstance(Timer timer) {
        for (timerInstance : timers) {
            if (timerInstance.name.equals(timer.name)) {
                return timerInstance
            }
        }
    }

    
    ///////////////////////////////////////////////////
    //// Methods for getting widths of ports and banks

    /**
     * For the specified width specification, return the width.
     * This may be for a bank of reactors within this reactor instance or
     * for a port of this reactor instance. If the argument is null, there
     * is no width specification, so return 1. Otherwise, evaluate the
     * width value by determining the value of any referenced parameters.
     * 
     * @param widthSpec The width specification.
     * 
     * @return The width, or -1 if it cannot be determined.
     */
    def width(WidthSpec widthSpec) {
        if (widthSpec.eContainer instanceof Instantiation && parent !== null) {
            // We need the instantiations list of the containing reactor,
            // not this one.
            return ASTUtils.width(widthSpec, parent.instantiations());
        }
        return ASTUtils.width(widthSpec, instantiations());
    }
    
    // ////////////////////////////////////////////////////
    // // Protected fields.
    
    /**
     * If this reactor is in a bank of reactors, then this member
     * refers to the reactor instance defining the bank.
     */
    protected ReactorInstance bank = null
    
    /** 
     * If this reactor instance is a placeholder for a bank of reactors,
     * as created by the new[width] ReactorClass() syntax, then this
     * list will be non-null and will contain the reactor instances in 
     * the bank.
     */
    protected List<ReactorInstance> bankMembers = null
    
    /** 
     * If this reactor is in a bank of reactors, its index, otherwise, -1
     * for an ordinary reactor and -2 for a placeholder for a bank of reactors.
     */
    protected int bankIndex = -1

    /** The generator that created this reactor instance. */
    protected GeneratorBase generator

    // ////////////////////////////////////////////////////
    // // Protected methods

    /** Create all the reaction instances of this reactor instance
     *  and record the dependencies and antidependencies
     *  between ports, actions, and timers and reactions.
     *  This also records the dependencies between reactions
     *  that follows from the order in which they are defined.
     */
    protected def createReactionInstances() {
        var reactions = this.definition.reactorClass.toDefinition.allReactions
        if (this.definition.reactorClass.toDefinition.reactions !== null) {
            
            var count = 0

            // Check for startup and shutdown triggers.
            for (Reaction reaction : reactions) {
                // Create the reaction instance.
                var reactionInstance = new ReactionInstance(reaction, this,
                    generator.isUnordered(reaction), count++)
                
                // Add the reaction instance to the map of reactions for this
                // reactor.
                this.reactions.add(reactionInstance);
                
                // Check for startup and shutdown triggers.
                var startupTrigger = null as TimerInstance;
                var shutdownTrigger = null as ActionInstance;
                for (trigger : reaction.triggers) {
                    if (trigger.isStartup) {
                        if (startupTrigger === null) {
                            // Null argument to TimerInstance makes this a startup trigger.
                            startupTrigger = new TimerInstance(null, this);
                            // Although this is a TimerInstance, it is treated differently from other timers.
                            // Do not add it to the list of timers.
                            // this.timers.add(startupTrigger);
                        }
                    } else if (trigger.isShutdown) {
                        if (shutdownTrigger === null) {
                             // Null argument to ActionInstance makes this a shutdown trigger.
                            shutdownTrigger = new ActionInstance(null, this);
                            // Although this is an ActionInstance, it is treated differently from other actions.
                            // Do not add it to the list of actions.
                            // this.actions.add(shutdownTrigger);
                        }
                    }
                }
                if (startupTrigger !== null) {
                    reactionInstance.triggers.add(startupTrigger);
                }
                if (shutdownTrigger !== null) {
                    reactionInstance.triggers.add(shutdownTrigger);
                }
            }
        }
    }

    /** Add to the specified destinations set all ports that receive data from the 
     *  specified source. This includes inputs and outputs at the same level 
     *  of hierarchy and input ports deeper in the hierarchy.
     *  It does not include inputs or outputs up the hierarchy (i.e., ones
     *  that are reached via any output port that it does return).
     *  @param source A port belonging to this reaction instance or one
     *   of its children.
     *  @param destinations The set of destinations to populate.
     */
    protected def void transitiveClosure(
            PortInstance source,
            LinkedHashSet<PortInstance> destinations
    ) {
        // Check that the specified port belongs to this reactor or one of its children.
        // The following assumes that the main reactor has no ports, or else
        // a NPE will occur.
        if (source.parent !== this && source.parent.parent !== this) {
            throw new Exception(
                "Internal error: port " + source + " does not belong to " +
                    this + " nor any of its children."
            )
        }
        // If the port is an input port, then include it in the result.
        if (source.isInput) {
            destinations.add(source)
        }
        var localDestinations = this.destinations.get(source)

        for (destination : localDestinations ?: emptyList) {
            destinations.add(destination)
            if (destination.isInput) {
                // Destination may have further destinations lower in the hierarchy.
                destination.parent.transitiveClosure(destination, destinations)
            } else if (destination.parent.parent !== null) {
                // Destination may have further destinations higher in the hierarchy.
                destination.parent.parent.transitiveClosure(destination, destinations)
            }
        }
    }

    /** Collect all reactions that have not been assigned a level and
     *  return the list.
     *  @param reactor The reactor for which to check reactions.
     *  @param result The list to add reactions to.
     *  @return The list of reactions without levels.
     */
    protected def LinkedList<ReactionInstance> reactionsWithoutLevels(
        ReactorInstance reactor,
        LinkedList<ReactionInstance> result
    ) {
        for (reaction : reactor.reactions) {
            if (reaction.level < 0) {
                result.add(reaction)
            }
        }
        for (child : reactor.children) {
            reactionsWithoutLevels(child, result)
        }
        result
    }

    ////////////////////////////////////////
    //// Private variables
    
    /** The nested list of instantiations that created this reactor instance. */
    var List<Instantiation> _instantiations;
}
