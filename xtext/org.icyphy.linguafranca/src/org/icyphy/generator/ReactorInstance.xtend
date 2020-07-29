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

package org.icyphy.generator

import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.List
import java.util.Set
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

import static extension org.icyphy.ASTUtils.*
import org.icyphy.graph.DirectedGraph

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

    /** Count of the number of chains seen so far. */
    int branchCount = 1
    
        /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent, or null for the main rector.
     *  @param generator The generator (for error reporting).
     */
    new(Instantiation definition, ReactorInstance parent, GeneratorBase generator) {
        // If the reactor is being instantiated with new[width], then pass -2
        // to the constructor, otherwise pass -1.
        this(definition, parent, generator, (definition.arraySpec !== null)? -2 : -1)
    }

    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent, or null for the main rector.
     *  @param generator The generator (for error reporting).
     *  @param reactorIndex -1 for an ordinary reactor, -2 for a
     *   placeholder for a bank of reactors, or the index of the
     *   reactor in a bank of reactors otherwise.
     */
    protected new(Instantiation definition, ReactorInstance parent, GeneratorBase generator, int reactorIndex) {
        super(definition, parent)
        this.generator = generator
        this.bankIndex = reactorIndex
        
        // If this reactor is actually a bank of reactors, then instantiate
        // each individual reactor in the bank and skip the rest of the
        // initialization for this reactor instance.
        if (reactorIndex === -2) {
            if (definition.arraySpec.ofVariableLength) {
                throw new Exception("Banks of reactors with variable length are not supported.")
            }
            var width = definition.arraySpec.length
            this.bankMembers = new ArrayList<ReactorInstance>(width)
            for (var index = 0; index < width; index++) {
                var childInstance = new ReactorInstance(definition, parent, generator, index)
                this.bankMembers.add(childInstance)
                childInstance.bank = this
                childInstance.bankIndex = index
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
            if (inputDecl.arraySpec === null) {
                this.inputs.add(new PortInstance(inputDecl, this))
            } else {
                this.inputs.add(new MultiportInstance(inputDecl, this, generator))
            }
        }

        // Instantiate outputs for this reactor instance
        for (outputDecl : definition.reactorClass.toDefinition.allOutputs) {
            if (outputDecl.arraySpec === null) {
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
        
        // In each connection statement, make as many connections as possible until
        // either the source ports or the destination ports have been used up.
        // If either is not used up, then subsequent connection statements will
        // pick up where this one left off.  If the source connection is used up,
        // then subsequent connection statements from this same source will start
        // over from the beginning.
        
        // The above principle is realized by the nextPort() function.

        // If after all connections in a reactor have been made, there remain
        // either destination ports or source ports that have not been used up,
        // issue a warning. Despite the warning, the generated code will run.
        // An output channel that has no corresponding input channel simply
        // results in discarding the data. An input channel that has no
        // corresponding output channel will always have its `is_present`
        // field evaluate to false.
        for (connection : definition.reactorClass.toDefinition.allConnections) {
            // Always check first whether there is a destination instance
            // ready to receive before checking whether there is a source
            // instance ready to send so that we can warn when not all sources
            // are connected (and hence data may be lost).
            var dstInstance = nextPort(connection.rightPort)
            if (dstInstance !== null) {
                var srcInstance = nextPort(connection.leftPort)
                while (dstInstance !== null && srcInstance !== null) {
                    connectPortInstances(connection, srcInstance, dstInstance)
                    dstInstance = nextPort(connection.rightPort)
                    if (dstInstance !== null) {
                        srcInstance = nextPort(connection.leftPort)
                    }
                }
            }
            // It is possible for dstInstance to be non-null here, which means
            // that a destination was found where there was no corresponding source.
            // Need to reverse the incrementing of the multiport channel and/or bank index.
            // Otherwise, the next connection to use this destination may skip over
            // a channel and/or bank.
            if (dstInstance !== null) {
                reverseIncrement(connection.rightPort, dstInstance)
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
            // Set of reactions that do not depend on other reactions at
            // a logical time instant.
            independentReactions = new HashSet<ReactionInstance>()
            reactionsWithDeadline = new HashSet<ReactionInstance>()

            // Add to the dependsOnReactions and dependentReactions
            // of each reaction instance all the
            // reaction instances that it depends on indirectly through ports or
            // that depend on this reaction. Collect all the reactions that
            // depend on no other reactions into the _independentReactions set.
            if (!collapseDependencies(this)) {
                throw new Exception(
                    "Model has no reactions at all. Nothing to do."
                )
            }
            val graph = this.getDependencyGraph()
            
            // Assign a level to each reaction. 
            // If there are cycles present in the graph, it will be detected here.
            assignLevels(graph)
            // Traverse the graph again, now starting from the leaves,
            // to set the chain IDs.
            assignChainIDs(graph, false) // FIXME: Temporarily disabled this.

            // Propagate any declared deadline upstream.
            propagateDeadlines()

            // FIXME: also record number of reactions.
            // We can use this to set the sizes of the queues.
        }
    }
    
    /** Data structure used by nextPort() to keep track of the next available bank. */
    var nextBankTable = new HashMap<VarRef,Integer>()

    /** Data structure used by nextPort() to keep track of the next available port. */
    var nextPortTable = new HashMap<PortInstance,Integer>()
    
    /**
     * Check for dangling connections.
     */
    def checkForDanglingConnections() {
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
                    generator.reportWarning(portInstance.definition,
                            "Not all port channels are connected.")
                }
            }
        }
    }
    
    /**
     * For the specified port reference, if it is a multiport, decrement
     * the multiport bank tracker by one. If it is also in a bank and the
     * decrement drops below zero, then decrement the bank also.
     * If it is a single port in a bank, then mark it unused and,
     * if it is also in a bank, decrement the bank counter.
     * @param portReference The reference to the port in a connect statement.
     * @param portInstance The port instance that was found (a single port).
     */
    def reverseIncrement(VarRef portReference, PortInstance portInstance) {
        if (portInstance.multiport !== null) {
            // Port is in a multiport.
            var portIndex = nextPortTable.get(portInstance.multiport)?:0
            if (portIndex > 0) {
                nextPortTable.put(portInstance.multiport, portIndex - 1)
            } else {
                // The portIndex is 0.
                // The port must be in a bank, so we have to decrement the bank count.
                var nextBank = nextBankTable.get(portReference) ?: 0
                if (nextBank > 0) {
                    nextBankTable.put(portReference, nextBank - 1)
                    // Have to also find the channel reference for the port in the
                    // previous bank and decrement that.
                    var reactor = this
                    if (portReference.container !== null) {
                        reactor = getChildReactorInstance(portReference.container)
                    }
                    if (reactor.bankMembers === null) {
                        // This should not occur.
                        generator.reportWarning(portReference,
                        '''INTERNAL ERROR 1: Port «portInstance.getFullName» should have had incremented its channel or bank index.''')
                    } else {
                        val memberReactor = reactor.bankMembers.get(nextBank - 1)
                        val port = memberReactor.lookupLocalPort(portReference.variable as Port)
                        val portIdx = nextPortTable.get(port)?:0
                        if (portIdx <= 0) {
                            // This should not occur.
                            generator.reportWarning(
                                portReference, '''INTERNAL ERROR 2: Port «portInstance.getFullName» should have had incremented its channel or bank index.''')
                        } else {
                            nextPortTable.put(port, portIdx - 1)
                        }
                    }
                } else {
                    // Bank index and multiport channel are both 0.
                    // This should not occur.
                    generator.reportWarning(portReference,
                    '''INTERNAL ERROR 3: Port «portInstance.getFullName» should have had incremented its channel or bank index.''')
                }
            }
        } else if (!portReference.isSource) {
            // Not a multiport.
            // Port may have been marked used when it was not.
            nextPortTable.put(portInstance, 0)
        }
    }
    
    /**
     * Given a VarRef for either the left or the right side of a connection
     * statement within this reactor instance, return the next available port instance
     * for that connection. If there are no more available ports, return null.
     * To realize a connection statement, make connections until one side or
     * the other of the connection statement causes this method to return null.
     * 
     * If this reactor is not a bank of reactors and the
     * port is not a multiport, then the returned port instance will simply
     * be the port referenced, which may be a port of this reactor or a port
     * of a contained reactor, unless that same port was returned by the previous
     * invocation of this method, in which case the returned value will be null.
     * 
     * If the port is a multiport, then this method will iterate through the ports
     * in the multiport, returning a new port each time it is called, until all ports
     * in the multiport have been returned. Then it will return null.
     * 
     * If this reactor is a bank of reactors, the this method will iterate through
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
        
        var portInstance = memberReactor.lookupLocalPort(portReference.variable as Port)
        
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
                    portInstance = memberReactor.lookupLocalPort(portReference.variable as Port)
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
                return null
            }
            // Mark this port used if it is not a source.
            if (!portReference.isSource) {
                nextPortTable.put(portInstance, 1)
            }
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
        if (srcInstance === null) {
            generator.reportError(connection.leftPort, "Invalid port.")
            return
        }
        if (dstInstance === null) {
            generator.reportError(connection.rightPort, "Invalid port.")
            return
        }
        srcInstance.dependentPorts.add(dstInstance)
        if (dstInstance.dependsOnPort !== null && dstInstance.dependsOnPort !== srcInstance) {
            generator.reportError(
                connection,
                "dstInstance port " + dstInstance.getFullName + " is already connected to " +
                    dstInstance.dependsOnPort.getFullName
            )
        }
        dstInstance.dependsOnPort = srcInstance
        var dstInstances = this.destinations.get(srcInstance)
        if (dstInstances === null) {
            dstInstances = new LinkedHashSet<PortInstance>()
            this.destinations.put(srcInstance, dstInstances)
        }
        dstInstances.add(dstInstance)
    }
    
    /** Override the base class to return the uniqueID of the bank rather
     *  than this member of the bank, if this is a member of a bank of reactors.
     *  @return An identifier for this instance that is guaranteed to be
     *   unique within the top-level parent.
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
    
    /** The contained instances, indexed by name. */
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
    
    /** Return the action instance within this reactor 
     *  instance corresponding to the specified action reference.
     *  @param action The action as an AST node.
     *  @return The corresponding action instance or null if the
     *   action does not belong to this reactor.
     */
    def getActionInstance(Action action) {
        for (actionInstance : actions) {
            if (actionInstance.name.equals(action.name)) {
                return actionInstance
            }
        }
    }

    /** Override the base class to append [index] if this reactpr
     *  is in a bank of reactors.
     *  @return The full name of this instance.
     */
    override String getFullName() {
        var result = super.getFullName()
        if (this.bankIndex >= 0) {
            result += "[" + this.bankIndex + "]"
        }
        result
    }

    /** Return the shutdown action within this reactor instance.
     *  @return The corresponding shutdown action instance or null
     *  if this reactor instance does not have a shutdown action.
     */
    def getShutdownAction() {
        for (actionInstance : actions) {
            if (actionInstance.isShutdown) {
                return actionInstance
            }
        }
    }

    /** Return the startup timer within this reactor instance.
     *  @return The corresponding startup timer instance or null
     *  if this reactor instance does not have a shutdown timer.
     */
    def getStartupTimer() {
        for (timerInstance : timers) {
            if (timerInstance.isStartup) {
                return timerInstance
            }
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

    /** Return the name of this instance as given by the definition.
     *  Note that is unique only relative to other instances with the same
     *  parent.
     *  @return The name of this instance.
     */
    override String getName() {
        this.definition.name
    }

    /** Return the parameter instance within this reactor 
     *  instance corresponding to the specified parameter definition.
     *  @param parameter The parameter as an AST node.
     *  @return The corresponding parameter instance or null if the
     *   action does not belong to this reactor.
     */
    def getParameterInstance(Parameter parameter) {
        for (paramInstance : parameters) {
            if (paramInstance.definition === parameter) {
                return paramInstance
            }
        }
    }

    /** Return the reaction instance within this reactor 
     *  instance corresponding to the specified reaction.
     *  @param reaction The reaction as an AST node.
     *  @return The corresponding reaction instance or null if the
     *   reaction does not belong to this reactor.
     */
    def getReactionInstance(Reaction reaction) {
        for (reactionInstance : reactions) {
            if (reactionInstance.definition === reaction) {
                return reactionInstance
            }
        }
    }

    /** Given a reference to a port belonging to this reactor
     *  instance, return the port instance.
     *  Return null if there is no such instance.
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
            var containerInstance = this.
                getChildReactorInstance(reference.container)
            if (containerInstance === null) return null
            return containerInstance.lookupLocalPort(reference.variable as Port)
        }
    }

    /** Return the timer instance within this reactor 
     *  instance corresponding to the specified timer reference.
     *  @param timer The timer as an AST node.
     *  @return The corresponding timer instance or null if the
     *   timer does not belong to this reactor.
     */
    def getTimerInstance(Timer timer) {
        for (timerInstance : timers) {
            if (timerInstance.name.equals(timer.name)) {
                return timerInstance
            }
        }
    }

    /** Return the trigger instances (input ports, timers, and actions
     *  that trigger reactions) belonging to this reactor instance.
     *  @return The trigger instances belonging to this reactor instance.
     */
    def getTriggers() {
        var triggers = new HashSet<TriggerInstance<Variable>>
        for (reaction : this.reactions) {
            triggers.addAll(reaction.triggers)
        }
        return triggers
    }

    /** Return the trigger instances (input ports, timers, and actions
     *  that trigger reactions) together the ports that the reaction reads
     *  but that don't trigger it.
     *  @return The trigger instances belonging to this reactor instance.
     */
    def getTriggersAndReads() {
        var triggers = new HashSet<TriggerInstance<Variable>>
        for (reaction : this.reactions) {
            triggers.addAll(reaction.triggers)
            triggers.addAll(reaction.reads)
        }
        return triggers
    }
    
    /** 
     * Given a port definition, return the port instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param port The port definition (a syntactic object in the AST).
     * @return A port instance, or null if there is none.
     */
    def PortInstance lookupLocalPort(Port port) {
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

    /** 
     * Given a parameter definition, return the parameter instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param port The parameter definition (a syntactic object in the AST).
     * @return A parameter instance, or null if there is none.
     */
    def ParameterInstance lookupLocalParameter(Parameter parameter) {
        return this.parameters.findFirst [
            it.definition === parameter
        ]
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

    /** Return the set of all ports that receive data from the 
     *  specified source. This includes inputs and outputs at the same level 
     *  of hierarchy and input ports deeper in the hierarchy.
     *  It does not include inputs or outputs up the hierarchy (i.e., ones
     *  that are reached via any output port that it does return).
     *  If the argument is an input port, then it is included in the result.
     *  No port will appear more than once in the result.
     *  @param source An output or input port.
     */
    def transitiveClosure(PortInstance source) {
        var result = new LinkedHashSet<PortInstance>();
        transitiveClosure(source, result);
        result
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
    
    /** Set of independent reactions. */
    protected Set<ReactionInstance> independentReactions // FIXME: use static var instead?

    protected Set<ReactionInstance> reactionsWithDeadline // FIXME: use static var instead?

    // ////////////////////////////////////////////////////
    // // Protected methods.
    /** Add to the specified set of reactions all the reactions
     *  that the specified port depends on.
     *  @param port The port.
     *  @param reactions The set of reactions to add to.
     */
    protected def void addReactionsPortDependsOn(
        PortInstance port,
        HashSet<ReactionInstance> reactions
    ) {
        reactions.addAll(port.dependsOnReactions)
        if (port.dependsOnPort !== null) {
            addReactionsPortDependsOn(port.dependsOnPort, reactions)
        }
    }

    /** Add to the specified set of reactions all the reactions
     *  that depend on the specified port.
     *  @param port The port.
     *  @param reactions The set of reactions to add to.
     */
    protected def void addReactionsDependingOnPort(
        PortInstance port,
        HashSet<ReactionInstance> reactions
    ) {
        reactions.addAll(port.dependentReactions)
        for (downstreamPort : port.dependentPorts) {
            addReactionsDependingOnPort(downstreamPort, reactions)
        }
    }

    /**
     * Extract a precedence graph from this reactor instance.
     * FIXME: this is somewhat redundant; it's probably better build the graph immediately.
     * At the very least we could just replace dependsOnReactions and dependentReactions
     * with a DirectedGraph<ReactionInstance>. 
     */
    protected def DirectedGraph<ReactionInstance> getDependencyGraph() {
        var graph = new DirectedGraph<ReactionInstance>()
        for (child : this.children) {
            graph.merge(child.dependencyGraph)
        }
        for (r : this.reactions) {
            for (dependency : r.dependsOnReactions) {
                graph.addEdge(r, dependency)
            }
            for (dependent: r.dependentReactions) {
                graph.addEdge(dependent, r)
            }
        }
        // Also add the independent nodes to the graph.
        if (this === main) {
            for (node : independentReactions) {
                graph.addNode(node)
            }
        }
        
        graph
    }

    /**
     * Propagate the given chain ID up one chain, propagate fresh IDs to
     * other upstream neighbors, and return a mask that overlaps with all the
     * chain IDs that were set upstream as a result of this method invocation.
     * The result of propagation is that each node has an ID that overlaps with
     * all upstream nodes that can reach it. This means that if a node has a
     * lower level than another node, but the two nodes do not have overlapping
     * chain IDs, the nodes are nonetheless independent from one another.
     * @param current The current node that is being visited.
     * @param graph The graph that encodes the dependencies between reactions.
     * @param chainID The current chain ID.
     */
    private def long propagateUp(ReactionInstance current,
        DirectedGraph<ReactionInstance> graph, long chainID) {
        val origins = graph.getOrigins(current)
        var mask = chainID
        var first = true
        var id = chainID
        // Iterate over the upstream neighbors by level from high to low.
        for (upstream : origins.sortBy[-level]) {
            if (first) {
                // Stay on the same chain the first time.
                first = false
            } else {
                // Create a new chain ID.
                id = 1 << (this.branchCount++ % 64)
            }
            // Propagate the ID upstream and add all returned bits
            // to the mask.
            mask = mask.bitwiseOr(
                    propagateUp(upstream, graph, id))
        }    
        
        // Apply the mask to the current chain ID.
        // If there were no upstream neighbors, the mask will
        // just be the chainID that was passed as an argument.
        current.chainID = current.chainID.bitwiseOr(mask)
        
        return mask
    }

    /**
     * Analyze the dependencies between reactions and assign each reaction
     * instance a chain identifier. The assigned IDs are such that the
     * bitwise conjunction between two chain IDs is always nonzero if there
     * exists a dependency between them. This facilitates runtime checks
     * to determine whether a reaction is ready to execute or has to wait
     * for an upstream reaction to complete.
     * @param graph The dependency graph.
     * @param optimize Whether or not make assignments that maximize the
     * amount of parallelism. If false, just assign 1 to every node.
     */
    protected def assignChainIDs(DirectedGraph<ReactionInstance> graph,
            boolean optimize) {
        val leafs = graph.leafNodes
        this.branchCount = 0
        if (optimize) {
            // Start propagation from the leaf nodes,
            // ordered by level from high to low.
            for (node : leafs.sortBy[-level]) {
                this.propagateUp(node, graph, 1 << (this.branchCount++ % 64))
            }    
        } else {
            for (node: graph.nodes) {
            node.chainID = 1
            }    
        }
    }

    /**
     * Analyze the dependencies between reactions and assign each reaction
     * instance a level.
     * This procedure is based on Kahn's algorithm for topological sorting.
     * Rather than establishing a total order, we establish a partial order.
     * In this order, the level of each reaction is the least upper bound of
     * the levels of the reactions it depends on.
     * If any cycles are present in the dependency graph, an exception is
     * thrown. This method should be called only on the top-level (main) reactor.
     */
    protected def assignLevels(DirectedGraph<ReactionInstance> dependencies) {       
        val graph = dependencies.copy
        var start = new ArrayList(graph.rootNodes)
        
        // All root nodes start with level 0.
        for (origin : start) {
            origin.level = 0
        }
        
        if (main.independentReactions.isEmpty) {
            throw new Exception(
                "Reactions form a cycle, where every reaction depends on another reaction!")
        } 

        while (!start.empty) {
            val origin = start.remove(0)
            val toRemove = new HashSet<ReactionInstance>()
            // Visit effect nodes.
            for (effect : graph.getEffects(origin)) {
                // Stage edge between origin and effect for removal.
                toRemove.add(effect)
                
                // Update level of downstream node.
                effect.level = Math.max(effect.level, origin.level+1)    
            }
            // Remove visited edges.
            for (effect : toRemove) {
                graph.removeEdge(effect, origin)
                // If the effect node has no more incoming edges,
                // then move it in the start set.
                if (graph.getOrigins(effect).size == 0) {
                    start.add(effect)
                }
            }
            
            // Remove visited origin.
            graph.removeNode(origin)
            
        }
        
        if (graph.nodeCount != 0) {
            generator.reportError(generator.mainDef, "Reactions form a cycle!");
            throw new Exception(
                "Reactions form a cycle!")
        }
    }
    
    /**
     * Iterate over all reactions that have a declared deadline, update their
     * inferred deadline, as well as the inferred deadlines of any reactions
     * upstream.
     */
    def propagateDeadlines() {
        // Assume the graph is acyclic.
        for (r : reactionsWithDeadline) {
            if (r.declaredDeadline !== null &&
                r.declaredDeadline.maxDelay !== null) {
                // Only lower the inferred deadline (which is set to the max by default),
                // if the declared deadline is earlier than the inferred one (based on
                // some other downstream deadline).
                if (r.deadline.isEarlierThan(r.declaredDeadline.maxDelay)) {
                    r.deadline = r.declaredDeadline.maxDelay
                }
            }
            propagateDeadline(r)
        }
    }
    
    /**
     * Given a reaction instance, propagate its inferred deadline upstream.
     * @param downstream Reaction instance with an inferred deadline that
     * is to be propagated upstream.
     */
    def void propagateDeadline(ReactionInstance downstream) {
        for (upstream : downstream.dependsOnReactions) {
            // Only lower the inferred deadline (which is set to the max by default),
            // if downstream deadline is earlier than the inferred one (based on
            // some other downstream deadline).
            if (downstream.deadline.isEarlierThan(upstream.deadline)) {
                upstream.deadline = downstream.deadline
            }
            propagateDeadline(upstream)
        }
    }

    /** Add to the dependsOnReactions and dependentReactions of each
     *  reaction all the other reactions defined by the specified
     *  reactor that the reaction depends on indirectly through ports or
     *  that depend on that reaction.
     *  This results in each reactionInstance knowing the complete
     *  set of reactions it depends on and that depend on it, thereby
     *  forming the dependence graph.
     *  If there are ultimately no reactions that that
     *  reaction depends on, then add that reaction to the list of
     *  independent reactions at the top level (the main reactor).
     *  If there are no reactions at all, then return false.
     *  Otherwise, return true.
     *  @param reactionInstance The reaction instance (must not be null).
     *  @return True if at least one reaction exists inside the reactor.
     */
    protected def boolean collapseDependencies(ReactorInstance reactor) {
        var result = false
        for (ReactionInstance reactionInstance : reactor.reactions) {
            result = true
            // Handle the ports that this reaction writes to.
            for (PortInstance port : reactionInstance.dependentPorts) {
                // Reactions that depend on a port that this reaction writes to
                // also, by transitivity, depend on this reaction instance.
                addReactionsDependingOnPort(port,
                    reactionInstance.dependentReactions)
            }
            // Handle the ports that this reaction reads from.
            for (PortInstance port : reactionInstance.dependsOnPorts) {
                // Reactions that write to such a port are also reactions that
                // that this reaction depends on, by transitivity.
                addReactionsPortDependsOn(port,
                    reactionInstance.dependsOnReactions)
            }
            // If, after all this, the reaction does not depend on any other
            // reactions, then it is an independent reaction.
            if (reactionInstance.dependsOnReactions.isEmpty()) {
                main.independentReactions.add(reactionInstance);
            }
            
            if (reactionInstance.definition.deadline !== null) {
                main.reactionsWithDeadline.add(reactionInstance)
            }
        }
        // Repeat for all children.
        for (child : reactor.children) {
            result = collapseDependencies(child) || result
        }
        return result
    }

    /** Create all the reaction instances of this reactor instance
     *  and record the dependencies and antidependencies
     *  between ports, actions, and timers and reactions.
     *  This also records the dependencies between reactions
     *  that follows from the order in which they are defined.
     */
    protected def createReactionInstances() {
        var reactions = this.definition.reactorClass.toDefinition.allReactions
        if (this.definition.reactorClass.toDefinition.reactions !== null) {
            var ReactionInstance previousReaction = null
            var count = 0

            for (Reaction reaction : reactions) {

                // Create the reaction instance.
                var reactionInstance = new ReactionInstance(reaction, this,
                    count++)
                    
                // If this is not an unordered reaction, then create a dependency
                // on any previously defined reaction.
                if (!generator.isUnordered(reaction)) {

                    // If there is an earlier reaction in this same reactor, then
                    // create a link in the dependence graph.
                    if (previousReaction !== null) {
                        previousReaction.dependentReactions.add(reactionInstance)
                        reactionInstance.dependsOnReactions.add(previousReaction)
                    }
                    previousReaction = reactionInstance;
                }
                // Add the reaction instance to the map of reactions for this
                // reactor.
                this.reactions.add(reactionInstance);
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
}
