/** A data structure for a reactor instance. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashSet
import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.Set
import org.eclipse.emf.common.util.EList
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

/** Representation of a runtime instance of a reactor.
 *  For the main reactor, which has no parent, once constructed,
 *  this object represents the entire Lingua Franca program.
 *  The constructor analyzes the graph of dependencies between
 *  reactions and throws exception if this graph is cyclic.
 *  @author Edward A. Lee, Marten Lohstroh
 */
class ReactorInstance extends NamedInstance<Instantiation> {
        
    /** Create a runtime instance from the specified definition
     *  and with the specified parent that instantiated it.
     *  This constructor should not be used directly, which is
     *  why it is protected.
     *  Instead, use GeneratorBase.reactorInstanceFactory().
     *  @param instance The Instance statement in the AST.
     *  @param parent The parent, or null for the main rector.
     *  @param generator The generator creating this instance.
     */
    protected new(Instantiation definition, ReactorInstance parent, GeneratorBase generator) {
        super(definition, parent)
        this.generator = generator
        
        // Instatiate parameters for this reactor instance.
        for (parameter: definition.reactorClass.parameters) {
            parameters.add(new ParameterInstance(parameter, this))
        }
        
        // Instantiate children for this reactor instance
        for (child : definition.reactorClass.instantiations) {
            var childInstance = generator.reactorInstanceFactory(child, this)
            this.children.add(childInstance)
        }
        
        // Instantiate inputs for this reactor instance
        for (inputDecl : definition.reactorClass.inputs) {
            this.inputs.add(new PortInstance(inputDecl, this))
        }
        
        // Instantiate outputs for this reactor instance
        for (outputDecl : definition.reactorClass.outputs) {
            this.outputs.add(new PortInstance(outputDecl, this))
        }
        
        // Instantiate timers for this reactor instance
        for (timerDecl : definition.reactorClass.timers) {
        	this.timers.add(new TimerInstance(timerDecl, this))
        }
        
        // Instantiate actions for this reactor instance
        for (actionDecl : definition.reactorClass.actions) {
        	this.actions.add(new ActionInstance(actionDecl, this))
        }
        
        // Populate destinations map and the connectivity information
        // in the port instances.
        // Note that this can only happen _after_ the children and 
        // port instances have been created.
        for (connection : definition.reactorClass.connections) {
            var srcInstance = this.getPortInstance(connection.leftPort)
            var dstInstance = this.getPortInstance(connection.rightPort)
            srcInstance.dependentPorts.add(dstInstance)
            dstInstance.dependsOnPorts.add(srcInstance)
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
            
            // Add to the dependsOnReactions and dependentReactions
            // of each reaction instance all the
            // reaction instances that it depends on indirectly through ports or
            // that depend on this reaction. Collect all the reactions that
            // depend on no other reactions into the _independentReactions set.
            collapseDependencies(this)
            
            // Analyze the dependency graph for reactions and assign
            // levels to each reaction.
            analyzeDependencies()

            // If there are reaction instances that have not been assigned
            // a level, throw an exception. There are cyclic dependencies.
            var reactionsInCycle = new LinkedList<ReactionInstance>()
            reactionsWithoutLevels(this, reactionsInCycle)
            if (!reactionsInCycle.isEmpty) {
                // There are cycles. Construct an error message.
                var inCycle = new LinkedList<String>
                for (reaction : reactionsInCycle) {
                    inCycle.add("reaction " + reaction.reactionIndex
                        + " in " + reaction.parent.getFullName
                    )
                }
                throw new Exception("Found cycles including: "
                    + inCycle.join(", ")
                )
            }
        }
    }

    //////////////////////////////////////////////////////
    //// Public fields.
    
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

    /** The timer instances belonging to this reactor instance. */
    public var timers = new LinkedList<TimerInstance>
    
    /** The trigger instances (input ports, timers, and actions
     *  that trigger reactions) belonging to this reactor instance.
     */
    public var triggers = new HashSet<TriggerInstance<Variable>>
    
    //////////////////////////////////////////////////////
    //// Public methods.

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
    
    /** Given a port definition, return the port instance
     *  corresponding to that definition, or null if there is
     *  no such instance.
     *  @param port The port definition (a syntactic object in the AST).
     *  @return A port instance, or null if there is none.
     */
    def lookupLocalPort(Port port) {
        // Search one of the inputs and outputs sets.
        var ports = null as LinkedList<PortInstance>
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
    
    //////////////////////////////////////////////////////
    //// Protected fields.

    /** The generator that created this reactor instance. */
    protected GeneratorBase generator

    /** Set of independent reactions. */
    protected Set<ReactionInstance> independentReactions

    //////////////////////////////////////////////////////
    //// Protected methods.

    /** Add to the specified set of reactions all the reactions
     *  that the specified port depends on.
     *  @param port The port.
     *  @param reactions The set of reactions to add to.
     */
    protected def void addReactionsPortDependsOn(
        PortInstance port, HashSet<ReactionInstance> reactions
    ) {
        reactions.addAll(port.dependsOnReactions)
        for (upstreamPort : port.dependsOnPorts) {
            addReactionsPortDependsOn(upstreamPort, reactions)
        }
    }

    /** Add to the specified set of reactions all the reactions
     *  that depend on the specified port.
     *  @param port The port.
     *  @param reactions The set of reactions to add to.
     */
    protected def void addReactionsDependingOnPort(
        PortInstance port, HashSet<ReactionInstance> reactions
    ) {
        reactions.addAll(port.dependentReactions)
        for (downstreamPort : port.dependentPorts) {
            addReactionsDependingOnPort(downstreamPort, reactions)
        }
    }
    
    /** Analyze the dependencies between reactions and assign levels.
     *  A reaction has level 0 if it has no dependence on any other reaction,
     *  i.e. it is the first reaction in a reactor and it is triggered by
     *  by an action or a timer, not a port. It has level 1 if it depends
     *  only on level 0 reactions. Etc.
     *  Throw an exception if there are cyclic dependencies and
     *  report the reactions that cannot be assigned levels and hence are
     *  part of the cycle.
     *  This should be called only on the top-level (main) reactor.
     */
    protected def void analyzeDependencies() {
        if (independentReactions.isEmpty()) {
            throw new Exception("Reactions form a cycle, where every reaction depends on another reaction!")
        }
        var candidatesForLevel = new LinkedList<ReactionInstance>()
        var level = 0
        for (reaction : independentReactions) {
            reaction.level = level
            candidatesForLevel.addAll(reaction.dependentReactions)
        }
        while (!candidatesForLevel.isEmpty) {
            level++
            candidatesForLevel = analyzeDependencies(candidatesForLevel, level)            
        }
    }
    
    /** For each reaction instance in the specified list, assign it the
     *  specified level if every reaction it depends on already has an
     *  assigned level less than the specified level. Otherwise, add it
     *  to a new list that is returned. For each reaction that is assigned
     *  a level, also add all its dependent reactions to the returned list.
     *  @param candidatesForLevel Candidate reactions for the specified level.
     *  @param level The specified level.
     *  @return Candidates for the next level.
     */
    protected def analyzeDependencies(
        LinkedList<ReactionInstance> candidatesForLevel,
        int level
    ) {
        var newCandidatesForLevel = new LinkedList<ReactionInstance>()
        for (reaction : candidatesForLevel) {
            var ready = true
            for (dependsOnReaction : reaction.dependsOnReactions) {
                if (dependsOnReaction.level < 0 // Not assigned.
                    || dependsOnReaction.level >= level // Should not occur.
                ) {
                    // Would be nice to break here, but xtend can't do that.
                    ready = false
                }
            }
            if (ready) {
                reaction.level = level
                newCandidatesForLevel.addAll(reaction.dependentReactions)
            } else {
                newCandidatesForLevel.add(reaction)
            }
        }
        newCandidatesForLevel
    }

    /** Add to the dependsOnReactions and dependentReactions all the
     *  reactions defined by the specified reactor that that
     *  reaction depends on indirectly through ports or
     *  that depend on that reaction.
     *  If there are ultimately no reactions that that
     *  reaction depends on, then add that reaction to the list of
     *  independent reactions at the top level (the main reactor).
     *  @param reactionInstance The reaction instance (must not be null).
     */
    protected def void collapseDependencies(ReactorInstance reactor) {
        for (ReactionInstance reactionInstance : reactor.reactions) {
            // Handle the ports that this reaction writes to.
            for (PortInstance port : reactionInstance.dependentPorts) {
                // Reactions that depend on a port that this reaction writes to
                // also, by transitivity, depend on this reaction instance.
                addReactionsDependingOnPort(port, reactionInstance.dependentReactions)
            }
            // Handle the ports that this reaction reads from.
            for (PortInstance port : reactionInstance.dependsOnPorts) {
                // Reactions that write to such a port are also reactions that
                // that this reaction depends on, by transitivity.
                addReactionsPortDependsOn(port, reactionInstance.dependsOnReactions)
            }
            // If, after all this, the reaction does not depend on any other
            // reactions, then it is an independent reaction.
            if (reactionInstance.dependsOnReactions.isEmpty()) {
                main.independentReactions.add(reactionInstance);
            }
        }
        // Repeat for all children.
        for (child : reactor.children) {
            collapseDependencies(child)
        }
    }
        
    /** Create all the reaction instances of this reactor instance
     *  and record the dependencies and antidependencies
     *  between ports, actions, and timers and reactions.
     *  This also records the dependencies between reactions
     *  that follows from the order in which they are defined.
     */
    protected def createReactionInstances() {
        var reactions = this.definition.reactorClass.reactions
        if (this.definition.reactorClass.reactions !== null) {
            var ReactionInstance previousReaction = null
            var count = 0
            for (Reaction reaction : reactions) {
                // Create the reaction instance.
                var reactionInstance = new ReactionInstance(reaction, this, count++)
                // If there is an earlier reaction in this same reactor, then
                // create a link in the dependence graph.
                if (previousReaction !== null) {
                    previousReaction.dependentReactions.add(reactionInstance)
                    reactionInstance.dependsOnReactions.add(previousReaction)
                }
                previousReaction = reactionInstance;
                // Add the reaction instance to the map of reactions for this
                // reactor.
                this.reactions.add(reactionInstance);

                // Establish (anti-)dependencies based
                // on what reactions use and produce.
                // Only consider inputs and outputs, ignore actions and timers.
                var EList<VarRef> deps = null;
                // First handle dependencies.
                // Collect all the triggering ports, actions, and timers
                // and also all the ports that the reaction reads.
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
                    for (VarRef dep : deps) {
                        var variable = dep.getVariable()
                        if (variable instanceof Port) {
                            var port = this.getPortInstance(dep)
                            triggers.add(port)
                            port.dependentReactions.add(reactionInstance)
                            reactionInstance.dependsOnPorts.add(port)
                        } else if (variable instanceof Action) {
                            var action = this.getActionInstance(variable)
                            triggers.add(action)
                            action.dependentReactions.add(reactionInstance)
                            reactionInstance.dependsOnActions.add(action)
                        } else if (variable instanceof Timer) {
                            var timer = this.getTimerInstance(variable)
                            triggers.add(timer)
                            timer.dependentReactions.add(reactionInstance)
                            reactionInstance.dependsOnTimers.add(timer)
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
                        if (antidep.variable instanceof Port) {
                            var port = this.getPortInstance(antidep);
                            port.dependsOnReactions.add(reactionInstance);
                            reactionInstance.dependentPorts.add(port);
                        }
                    }
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
    protected def void transitiveClosure(PortInstance source, LinkedHashSet<PortInstance> destinations) {
        // Check that the specified port belongs to this reactor or one of its children.
        // The following assumes that the main reactor has no ports, or else
        // a NPE will occur.
        if (source.parent !== this && source.parent.parent !== this) {
            throw new Exception("Internal error: port " + source + " does not belong to "
                + this + " nor any of its children."
            )
        }
        // If the port is an input port, then include it in the result.
        if (source.definition instanceof Input) {
            destinations.add(source)
        }
        var localDestinations = this.destinations.get(source)
        
        for (destination : localDestinations?:emptyList) {
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