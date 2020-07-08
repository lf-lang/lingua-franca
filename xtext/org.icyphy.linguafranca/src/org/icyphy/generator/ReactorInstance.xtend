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
import java.util.HashSet
import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.Set
import org.icyphy.graph.DirectedGraph
import org.icyphy.linguaFranca.Action
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

    int branchCount = 1

    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent, or null for the main rector.
     *  @param generator The generator (for error reporting).
     */
    new(Instantiation definition, ReactorInstance parent, GeneratorBase generator) {
        super(definition, parent)
        this.generator = generator

        // Apply overrides and instantiate parameters for this reactor instance.
        for (parameter : definition.reactorClass.allParameters) {
            this.parameters.add(new ParameterInstance(parameter, this))
        }

        // Instantiate children for this reactor instance
        for (child : definition.reactorClass.allInstantiations) {
            var childInstance = new ReactorInstance(child, this, generator)
            this.children.add(childInstance)
        }

        // Instantiate inputs for this reactor instance
        for (inputDecl : definition.reactorClass.allInputs) {
            if (inputDecl.arraySpec === null) {
                this.inputs.add(new PortInstance(inputDecl, this))
            } else {
                this.inputs.add(new MultiportInstance(inputDecl, this, generator))
            }
        }

        // Instantiate outputs for this reactor instance
        for (outputDecl : definition.reactorClass.allOutputs) {
            this.outputs.add(new PortInstance(outputDecl, this))
        }

        // Instantiate timers for this reactor instance
        for (timerDecl : definition.reactorClass.allTimers) {
            this.timers.add(new TimerInstance(timerDecl, this))
        }

        // Instantiate actions for this reactor instance
        for (actionDecl : definition.reactorClass.allActions) {
            this.actions.add(new ActionInstance(actionDecl, this))
        }

        // Populate destinations map and the connectivity information
        // in the port instances.
        // Note that this can only happen _after_ the children and 
        // port instances have been created.
        for (connection : definition.reactorClass.allConnections) {
            var srcInstance = this.getPortInstance(connection.leftPort)
            var dstInstance = this.getPortInstance(connection.rightPort)
            
            // If the right side of the connection has the form port[i],
            // then use the specific port, not the multiport.
            if (connection.rightPort.variableArraySpec !== null) {
                val width = (dstInstance as MultiportInstance).instances.size
                val index = connection.rightPort.variableArraySpec.length
                if (index >= width) {
                    generator.reportError(connection.rightPort, "Index out of range.")
                }
                dstInstance = (dstInstance as MultiportInstance).instances.get(index)
            }
            // If the left side of the connection has the form port[i],
            // then use the specific port, not the multiport.
            if (connection.leftPort.variableArraySpec !== null) {
                val width = (dstInstance as MultiportInstance).instances.size
                val index = connection.leftPort.variableArraySpec.length
                if (index >= width) {
                    generator.reportError(connection.leftPort, "Index out of range.")
                }
                dstInstance = (dstInstance as MultiportInstance).instances.get(index)
            }
            
            srcInstance.dependentPorts.add(dstInstance)
            if (dstInstance.dependsOnPort !== null &&
                dstInstance.dependsOnPort !== srcInstance) {
                generator.reportError(connection,
                    "Destination port " + dstInstance.getFullName +
                        " is already connected to " +
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
            var containerInstance = this.
                getChildReactorInstance(reference.container)
            return containerInstance.lookupLocalPort(reference.variable as Port)
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

    /** Add to the dependsOnReactions and dependentReactions all the
     *  reactions defined by the specified reactor that that
     *  reaction depends on indirectly through ports or
     *  that depend on that reaction.
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
        var reactions = this.definition.reactorClass.allReactions
        if (this.definition.reactorClass.reactions !== null) {
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
        if (source.definition instanceof Input) {
            destinations.add(source)
        }
        var localDestinations = this.destinations.get(source)

        for (destination : localDestinations ?: emptyList) {
            destinations.add(destination)
            destination.parent.transitiveClosure(destination, destinations)
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
