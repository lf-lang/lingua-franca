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

package org.lflang.generator;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.generator.TriggerInstance.BuiltinTriggerVariable;
import org.lflang.lf.Action;
import org.lflang.lf.Connection;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthSpec;


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
public class ReactorInstance extends NamedInstance<Instantiation> {

    /**
     * Create a new instantiation hierarchy that starts with the given top-level reactor.
     * @param reactor The top-level reactor.
     * @param reporter The error reporter.
     */
    public ReactorInstance(Reactor reactor, ErrorReporter reporter) {
        this(ASTUtils.createInstantiation(reactor), null, reporter, -1, null);
    }

    /**
     * Create a new instantiation hierarchy that starts with the given top-level reactor
     * but only creates contained reactors up to the specified depth.
     * @param reactor The top-level reactor.
     * @param reporter The error reporter.
     * @param desiredDepth The depth to which to go, or -1 to construct the full hierarchy.
     */
    public ReactorInstance(Reactor reactor, ErrorReporter reporter, int desiredDepth) {
        this(ASTUtils.createInstantiation(reactor), null, reporter, desiredDepth, null);
    }

    /**
     * Create a new instantiation hierarchy that starts with the given reactor.
     * @param reactor The top-level reactor.
     * @param reporter The error reporter.
     * @param unorderedReactions A list of reactions that should be treated as unordered.
     */
    public ReactorInstance(Reactor reactor, ErrorReporter reporter, Set<Reaction> unorderedReactions) {
        this(ASTUtils.createInstantiation(reactor), null, reporter, -1, unorderedReactions);
    }

    /**
     * Create a new instantiation with the specified parent.
     * This constructor is here to allow for unit tests.
     * It should not be used for any other purpose.
     * @param reactor The top-level reactor.
     * @param reporter The error reporter.
     * @param parent The parent reactor instance.
     */
    public ReactorInstance(Reactor reactor, ErrorReporter reporter, ReactorInstance parent) {
        this(ASTUtils.createInstantiation(reactor), parent, reporter, -1, null);
    }

    //////////////////////////////////////////////////////
    //// Public fields.

    /** The action instances belonging to this reactor instance. */
    public final List<ActionInstance> actions = new ArrayList<ActionInstance>();

    /**
     * The contained reactor instances, in order of declaration.
     * For banks of reactors, this includes both the bank definition
     * Reactor (which has bankIndex == -2) followed by each of the
     * bank members (which have bankIndex >= 0).
     */
    public final List<ReactorInstance> children = new ArrayList<ReactorInstance>();

    /** The input port instances belonging to this reactor instance. */
    public final List<PortInstance> inputs = new ArrayList<PortInstance>();

    /** The output port instances belonging to this reactor instance. */
    public final List<PortInstance> outputs = new ArrayList<PortInstance>();

    /** The parameters of this instance. */
    public final List<ParameterInstance> parameters = new ArrayList<ParameterInstance>();

    /** List of reaction instances for this reactor instance. */
    public final List<ReactionInstance> reactions = new ArrayList<ReactionInstance>();

    /** The timer instances belonging to this reactor instance. */
    public final List<TimerInstance> timers = new ArrayList<TimerInstance>();

    /** The reactor definition in the AST. */
    public final Reactor reactorDefinition;

    /** Indicator that this reactor has itself as a parent, an error condition. */
    public final boolean recursive;

    //////////////////////////////////////////////////////
    //// Public methods.
    
    /**
     * Assign levels to all reactions within the same root as this
     * reactor. The level of a reaction r is equal to the length of the
     * longest chain of reactions that must have the opportunity to
     * execute before r at each logical tag. This fails and returns
     * false if a causality cycle exists.
     * 
     * This method uses a variant of Kahn's algorithm, which is linear
     * in V + E, where V is the number of vertices (reactions) and E
     * is the number of edges (dependencies between reactions).
     * 
     * @return An empty graph if successful and otherwise a graph
     *  with runtime reaction instances that form cycles.
     */
    public ReactionInstanceGraph assignLevels() {
        if (depth != 0) return root().assignLevels();
        if (cachedReactionLoopGraph == null) {
            cachedReactionLoopGraph = new ReactionInstanceGraph(this);
        }
        return cachedReactionLoopGraph;
    }
    
    /** 
     * Return the instance of a child rector created by the specified
     * definition or null if there is none.
     * @param definition The definition of the child reactor ("new" statement).
     */
    public ReactorInstance getChildReactorInstance(Instantiation definition) {
        for (ReactorInstance child : this.children) {
            if (child.definition == definition) {
                return child;
            }
        }
        return null;
    }
    
    /**
     * Clear any cached data.
     * This is useful if a mutation has been realized.
     */
    public void clearCaches() {
        cachedReactionLoopGraph = null;
        totalNumberOfReactionsCache = -1;
    }

    /**
     * Return an integer is equal to one plus the total number of reactor instances
     * (including bank members) that have been instantiated before this one
     * within the same parent.  If the number of instances cannot be determined,
     * this will be 0.
     * 
     * When this number can be determined, it can be used, for example,
     * to index into an array of data structures
     * representing instances of reactors in generated code.
     * This assumes that index 0 refers to the parent, hence the "one plus."
     */
    public int getIndexOffset() {
        return indexOffset;
    }
    
    /**
     * Return the specified input by name or null if there is no such input.
     * @param name The input name.
     */
    public PortInstance getInput(String name) {
        for (PortInstance port: inputs) {
            if (port.getName().equals(name)) {
                return port;
            }
        }
        return null;
    }

    /** 
     * Override the base class to append [i_d], where d is the depth,
     * if this reactor is in a bank of reactors.
     * @return The name of this instance.
     */
    @Override
    public String getName() {
        return this.definition.getName();
    }

    /**
     * Return one plus the number of contained reactor instances in this
     * reactor, if this can be determined, or -1 if not. This is 1 plus 
     * the total number of reactors in any contained reactor instances,
     * taking into account their bank widths if necessary.
     */
    public int getNumReactorInstances() {
        return numReactorInstances;
    }
    
    /**
     * Return the specified output by name or null if there is no such output.
     * @param name The output name.
     */
    public PortInstance getOutput(String name) {
        for (PortInstance port: outputs) {
            if (port.getName().equals(name)) {
                return port;
            }
        }
        return null;
    }
    
    /**
     * Return a parameter matching the specified name if the reactor has one
     * and otherwise return null.
     * @param name The parameter name.
     */
    public ParameterInstance getParameter(String name) {
        for (ParameterInstance parameter: parameters) {
            if (parameter.getName().equals(name)) {
                return parameter;
            }
        }
        return null;
    }
    
    /**
     * Return the startup trigger or null if not used in any reaction.
     */
    public TriggerInstance<BuiltinTriggerVariable> getStartupTrigger() {
        return startupTrigger;
    }

    /**
     * Return the shutdown trigger or null if not used in any reaction.
     */
    public TriggerInstance<BuiltinTriggerVariable> getShutdownTrigger() {
        return shutdownTrigger;
    }
    
    /**
     * Return the total number of reactor instances associated with
     * this reactor, if it can be determined, or -1 if not. This is equal
     * to the result of getNumReactorInstances() times the bank width,
     * as returned by width().
     */
    public int getTotalNumReactorInstances() {
        if (width < 0 || numReactorInstances < 0) return -1;
        return width * numReactorInstances;
    }
    
    /** 
     * Return the trigger instances (input ports, timers, and actions
     * that trigger reactions) belonging to this reactor instance.
     */
    public Set<TriggerInstance<? extends Variable>> getTriggers() {
        // FIXME: Cache this.
        var triggers = new LinkedHashSet<TriggerInstance<? extends Variable>>();
        for (ReactionInstance reaction : this.reactions) {
            triggers.addAll(reaction.triggers);
        }
        return triggers;
    }

    /** 
     * Return the trigger instances (input ports, timers, and actions
     * that trigger reactions) together the ports that the reaction reads
     * but that don't trigger it.
     * 
     * @return The trigger instances belonging to this reactor instance.
     */
    public Set<TriggerInstance<? extends Variable>> getTriggersAndReads() {
        // FIXME: Cache this.
        var triggers = new LinkedHashSet<TriggerInstance<? extends Variable>>();
        for (ReactionInstance reaction : this.reactions) {
            triggers.addAll(reaction.triggers);
            triggers.addAll(reaction.reads);
        }
        return triggers;
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
    public Integer initialIntParameterValue(Parameter parameter) {
        return ASTUtils.initialValueInt(parameter, instantiations());
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
    public List<Value> initialParameterValue(Parameter parameter) {
        return ASTUtils.initialValue(parameter, instantiations());
    }

    /**
     * Return a list of Instantiation objects for evaluating parameter
     * values.  The first object in the list is the AST Instantiation
     * that created this reactor instance, the second is the AST instantiation
     * that created the containing reactor instance, and so on until there
     * are no more containing reactor instances. This will return an empty
     * list if this reactor instance is at the top level (is main).
     */
    public List<Instantiation> instantiations() {
        if (_instantiations == null) {
            _instantiations = new ArrayList<Instantiation>();
            if (definition != null) {
                _instantiations.add(definition);
                if (parent != null) {
                    _instantiations.addAll(parent.instantiations());
                }
            }
        }
        return _instantiations;
    }

    /**
     * Returns true if this is a bank of reactors.
     * @return true if a reactor is a bank, false otherwise
     */
    public boolean isBank() {
        return (definition.getWidthSpec() != null);
    }

    /**
     * Returns whether this is a main or federated reactor.
     * @return true if reactor definition is marked as main or federated, false otherwise.
     */
    public boolean isMainOrFederated() {
        return reactorDefinition != null 
                && (reactorDefinition.isMain() || reactorDefinition.isFederated());
    }
    
    ///////////////////////////////////////////////////
    //// Methods for finding instances in this reactor given an AST node.
    
    /** 
     * Return the action instance within this reactor 
     * instance corresponding to the specified action reference.
     * @param action The action as an AST node.
     * @return The corresponding action instance or null if the
     *  action does not belong to this reactor.
     */
    public ActionInstance lookupActionInstance(Action action) {
        for (ActionInstance actionInstance : actions) {
            if (actionInstance.definition == action) {
                return actionInstance;
            }
        }
        return null;
    }

    /** 
     * Given a parameter definition, return the parameter instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param parameter The parameter definition (a syntactic object in the AST).
     * @return A parameter instance, or null if there is none.
     */
    public ParameterInstance lookupParameterInstance(Parameter parameter) {
        for (ParameterInstance param : parameters) {
            if (param.definition == parameter) {
                return param;
            }
        }
        return null;
    }
    
    /** 
     * Given a port definition, return the port instance
     * corresponding to that definition, or null if there is
     * no such instance.
     * @param port The port definition (a syntactic object in the AST).
     * @return A port instance, or null if there is none.
     */
    public PortInstance lookupPortInstance(Port port) {
        // Search one of the inputs and outputs sets.
        List<PortInstance> ports = null;
        if (port instanceof Input) {
            ports = this.inputs;
        } else if (port instanceof Output) {
            ports = this.outputs;
        }
        for (PortInstance portInstance : ports) {
            if (portInstance.definition == port) {
                return portInstance;
            }
        }
        return null;
    }

    /** 
     * Given a reference to a port belonging to this reactor
     * instance, return the port instance.
     * Return null if there is no such instance.
     * @param reference The port reference.
     * @return A port instance, or null if there is none.
     */
    public PortInstance lookupPortInstance(VarRef reference) {
        if (!(reference.getVariable() instanceof Port)) {
            // Trying to resolve something that is not a port
            return null;
        }
        if (reference.getContainer() == null) {
            // Handle local reference
            return lookupPortInstance((Port) reference.getVariable());
        } else {
            // Handle hierarchical reference
            var containerInstance = getChildReactorInstance(reference.getContainer());
            if (containerInstance == null) return null;
            return containerInstance.lookupPortInstance((Port) reference.getVariable());
        }
    }

    /** 
     * Return the reaction instance within this reactor 
     * instance corresponding to the specified reaction.
     * @param reaction The reaction as an AST node.
     * @return The corresponding reaction instance or null if the
     *  reaction does not belong to this reactor.
     */
    public ReactionInstance lookupReactionInstance(Reaction reaction) {
        for (ReactionInstance reactionInstance : reactions) {
            if (reactionInstance.definition == reaction) {
                return reactionInstance;
            }
        }
        return null;
    }
    
    /**
     * Return the reactor instance within this reactor
     * that has the specified instantiation. Note that this
     * may be a bank of reactors. Return null if there
     * is no such reactor instance.
     */
    public ReactorInstance lookupReactorInstance(Instantiation instantiation) {
        for (ReactorInstance reactorInstance : children) {
            if (reactorInstance.definition == instantiation) {
                return reactorInstance;
            }
        }
        return null;
    }
    
    /** 
     * Return the timer instance within this reactor 
     * instance corresponding to the specified timer reference.
     * @param timer The timer as an AST node.
     * @return The corresponding timer instance or null if the
     *  timer does not belong to this reactor.
     */
    public TimerInstance lookupTimerInstance(Timer timer) {
        for (TimerInstance timerInstance : timers) {
            if (timerInstance.definition == timer) {
                return timerInstance;
            }
        }
        return null;
    }

    /** 
     * Return a descriptive string.
     */
    @Override
    public String toString() {
        return "ReactorInstance " + getFullName();
    }
    
    /**
     * Return the total number of reactions in this reactor
     * and all its contained reactors.
     */
    public int totalNumberOfReactions() {
        if (totalNumberOfReactionsCache >= 0) return totalNumberOfReactionsCache;
        totalNumberOfReactionsCache = reactions.size();
        for (ReactorInstance containedReactor : children) {
            totalNumberOfReactionsCache += containedReactor.totalNumberOfReactions();
        }
        return totalNumberOfReactionsCache;
    }

    /** 
     * Return the set of all ports that receive data from the 
     * specified source. This includes inputs and outputs at the same level 
     * of hierarchy and input ports deeper in the hierarchy.
     * It also includes inputs or outputs up the hierarchy (i.e., ones
     * that are reached via any output port).
     * If the argument is an input port, then it is included in the result.
     * No port will appear more than once in the result.
     * 
     * @param source An output or input port.
     */
    public Set<PortInstance> transitiveClosure(PortInstance source) {
        var result = new LinkedHashSet<PortInstance>();
        transitiveClosure(source, result);
        return result;
    }
    
    //////////////////////////////////////////////////////
    //// Protected fields.

    /** The generator that created this reactor instance. */
    protected ErrorReporter reporter; // FIXME: This accumulates a lot of redundant references

    /** The startup trigger. Null if not used in any reaction. */
    protected TriggerInstance<BuiltinTriggerVariable> startupTrigger = null;

    /** The shutdown trigger. Null if not used in any reaction. */
    protected TriggerInstance<BuiltinTriggerVariable> shutdownTrigger = null;

    /**
     * The LF syntax does not currently support declaring reactions unordered,
     * but unordered reactions are created in the AST transformations handling
     * federated communication and after delays. Unordered reactions can execute
     * in any order and concurrently even though they are in the same reactor.
     * FIXME: Remove this when the language provides syntax.
     */
    protected Set<Reaction> unorderedReactions = new LinkedHashSet<Reaction>();

    /** The nested list of instantiations that created this reactor instance. */
    protected List<Instantiation> _instantiations;

    //////////////////////////////////////////////////////
    //// Protected methods.
    
    /** 
     * Create all the reaction instances of this reactor instance
     * and record the dependencies and antidependencies
     * between ports, actions, and timers and reactions.
     * This also records the dependencies between reactions
     * that follows from the order in which they are defined.
     */
    protected void createReactionInstances() {
        List<Reaction> reactions = ASTUtils.allReactions(reactorDefinition);
        if (reactions != null) {
            int count = 0;

            // Check for startup and shutdown triggers.
            for (Reaction reaction : reactions) {
                // Create the reaction instance.
                var reactionInstance = new ReactionInstance(reaction, this,
                    unorderedReactions.contains(reaction), count++);
                
                // Add the reaction instance to the map of reactions for this
                // reactor.
                this.reactions.add(reactionInstance);
            }
        }
    }

    /**
     * Returns the startup trigger or create a new one if none exists.
     */
    protected TriggerInstance<? extends Variable> getOrCreateStartup(TriggerRef trigger) {
        if (startupTrigger == null) {
            startupTrigger = new TriggerInstance<BuiltinTriggerVariable>(
                    TriggerInstance.BuiltinTrigger.STARTUP, trigger, this);
        }
        return startupTrigger;
    }
    
    /**
     * Returns the shutdown trigger or create a new one if none exists.
     */
    protected TriggerInstance<? extends Variable> getOrCreateShutdown(TriggerRef trigger) {
        if (shutdownTrigger == null) {
            shutdownTrigger = new TriggerInstance<BuiltinTriggerVariable>(
                    TriggerInstance.BuiltinTrigger.SHUTDOWN, trigger, this);
        }
        return shutdownTrigger;
    }
    
    /** 
     * Add to the specified destinations set all ports that receive data from the 
     * specified source. This includes inputs and outputs at the same level 
     * of hierarchy and input ports deeper in the hierarchy.
     * It also includes inputs or outputs up the hierarchy (i.e., ones
     * that are reached via any output port).
     * @param source A port belonging to this reaction instance or one
     *  of its children.
     * @param destinations The set of destinations to populate.
     */
    protected void transitiveClosure(
            PortInstance source,
            LinkedHashSet<PortInstance> destinations
    ) {
        // Check that the specified port belongs to this reactor or one of its children.
        // The following assumes that the main reactor has no ports, or else
        // a NPE will occur.
        if (source.parent != this && source.parent.parent != this) {
            throw new InvalidSourceException(
                "Internal error: port " + source + " does not belong to " +
                    this + " nor any of its children."
            );
        }
        // If the port is an input port, then include it in the result.
        if (source.isInput()) {
            destinations.add(source);
        }
        for (Range<PortInstance> dst : source.dependentPorts) {
            PortInstance destination = dst.instance;
            destinations.add(destination);
            if (destination.isInput()) {
                // Destination may have further destinations lower in the hierarchy.
                destination.parent.transitiveClosure(destination, destinations);
            } else if (destination.parent.parent != null) {
                // Destination may have further destinations higher in the hierarchy.
                destination.parent.parent.transitiveClosure(destination, destinations);
            }
        }
    }
    
    ////////////////////////////////////////
    //// Private constructors
    
    /**
     * Create a runtime instance from the specified definition
     * and with the specified parent that instantiated it.
     * @param definition The instantiation statement in the AST.
     * @param parent The parent, or null for the main rector.
     * @param reporter An error reporter.
     * @param desiredDepth The depth to which to expand the hierarchy.
     * @param unorderedReactions A list of reactions that should be treated as unordered.
     *  It can be passed as null.
     */
    private ReactorInstance(
            Instantiation definition, 
            ReactorInstance parent,
            ErrorReporter reporter,
            int desiredDepth,
            Set<Reaction> unorderedReactions) {
        super(definition, parent);
        this.reporter = reporter;
        this.reactorDefinition = ASTUtils.toDefinition(definition.getReactorClass());
        
        if (unorderedReactions != null) {
            this.unorderedReactions = unorderedReactions;
        }
        
        // check for recursive instantiation
        var currentParent = parent;
        var foundSelfAsParent = false;
        do {
            if (currentParent != null) {
                if (currentParent.reactorDefinition == this.reactorDefinition) {
                    foundSelfAsParent = true;
                    currentParent = null; // break
                } else {
                    currentParent = currentParent.parent;
                }
            }
        } while(currentParent != null);
        
        this.recursive = foundSelfAsParent;
        if (recursive) {
            reporter.reportError(definition, "Recursive reactor instantiation.");
        }
                
        // If the reactor definition is null, give up here. Otherwise, diagram generation
        // will fail an NPE.
        if (reactorDefinition == null) {
            reporter.reportError(definition, "Reactor instantiation has no matching reactor definition.");
            return;
        }
        
        setInitialWidth();
        
        // Apply overrides and instantiate parameters for this reactor instance.
        for (Parameter parameter : ASTUtils.allParameters(reactorDefinition)) {
            this.parameters.add(new ParameterInstance(parameter, this));
        }

        // Instantiate inputs for this reactor instance
        for (Input inputDecl : ASTUtils.allInputs(reactorDefinition)) {
            this.inputs.add(new PortInstance(inputDecl, this, reporter));
        }

        // Instantiate outputs for this reactor instance
        for (Output outputDecl : ASTUtils.allOutputs(reactorDefinition)) {
            this.outputs.add(new PortInstance(outputDecl, this, reporter));
        }

        // Do not process content (except interface above) if recursive
        if (!recursive && (desiredDepth < 0 || this.depth < desiredDepth)) {
            // Instantiate children for this reactor instance.
            // While doing this, assign an index offset to each.
            int offset = 1;
            for (Instantiation child : ASTUtils.allInstantiations(reactorDefinition)) {
                var childInstance = new ReactorInstance(
                    child, 
                    this, 
                    reporter, 
                    desiredDepth,
                    this.unorderedReactions
                );
                this.children.add(childInstance);
                int numChildReactors = childInstance.getTotalNumReactorInstances();
                if (this.numReactorInstances >= 0 && numChildReactors > 0) {
                    this.numReactorInstances += numChildReactors;
                    
                    childInstance.indexOffset = offset;
                    // Next child will have an offset augmented by the total number of
                    // reactor instances in this one.
                    offset += childInstance.getTotalNumReactorInstances();
                } else {
                    numReactorInstances = -1;
                }
            }

            // Instantiate timers for this reactor instance
            for (Timer timerDecl : ASTUtils.allTimers(reactorDefinition)) {
                this.timers.add(new TimerInstance(timerDecl, this));
            }

            // Instantiate actions for this reactor instance
            for (Action actionDecl : ASTUtils.allActions(reactorDefinition)) {
                this.actions.add(new ActionInstance(actionDecl, this));
            }

            establishPortConnections();

            // Create the reaction instances in this reactor instance.
            // This also establishes all the implied dependencies.
            // Note that this can only happen _after_ the children, 
            // port, action, and timer instances have been created.
            createReactionInstances();
        }
    }
    
    //////////////////////////////////////////////////////
    //// Private methods.

    /**
     * Connect the given left port range to the given right port range.
     * This method consolidates the interleaved state on the destination
     * side.  That is, it sets the interleaved state of the destination
     * to the exclusive OR of the interleaved state of the two ranges,
     * and sets the interleaved state of the source to false.
     * @param src The source range.
     * @param dst The destination range.
     */
    private void connectPortInstances(
            Range<PortInstance> src,
            Range<PortInstance> dst
    ) {
        src.instance.dependentPorts.add(dst);
        dst.instance.dependsOnPorts.add(src);
    }

    /**
     * Populate connectivity information in the port instances.
     * Note that this can only happen _after_ the children and port instances have been created.
     * Unfortunately, we have to do some complicated things here
     * to support multiport-to-multiport, multiport-to-bank,
     * and bank-to-multiport communication.  The principle being followed is:
     * in each connection statement, for each port instance on the left,
     * connect to the next available port on the right.
     */
    private void establishPortConnections() {
        for (Connection connection : ASTUtils.allConnections(reactorDefinition)) {
            List<Range<PortInstance>> leftPorts = listPortInstances(connection.getLeftPorts(), connection);
            Iterator<Range<PortInstance>> srcRanges = leftPorts.iterator();
            Iterator<Range<PortInstance>> dstRanges 
                    = listPortInstances(connection.getRightPorts(), connection).iterator();
            
            // Check for empty lists.
            if (!srcRanges.hasNext()) {
                if (dstRanges.hasNext()) {
                    reporter.reportWarning(connection, "No sources to provide inputs.");
                }
                return;
            } else if (!dstRanges.hasNext()) {
                reporter.reportWarning(connection, "No destination. Outputs will be lost.");
                return;
            }
            
            Range<PortInstance> src = srcRanges.next();
            Range<PortInstance> dst = dstRanges.next();

            while(true) {
                if (dst.width == src.width) {
                    connectPortInstances(src, dst);
                    if (!dstRanges.hasNext()) {
                        if (srcRanges.hasNext()) {
                            // Should not happen (checked by the validator).
                            reporter.reportWarning(connection, 
                                    "Source is wider than the destination. Outputs will be lost.");
                        }
                        break;
                    }
                    if (!srcRanges.hasNext()) {
                        if (connection.isIterated()) {
                            srcRanges = leftPorts.iterator();
                        } else {
                            if (dstRanges.hasNext()) {
                                // Should not happen (checked by the validator).
                                reporter.reportWarning(connection, 
                                        "Destination is wider than the source. Inputs will be missing.");
                            }
                            break;
                        }
                    }
                    dst = dstRanges.next();
                    src = srcRanges.next();
                } else if (dst.width < src.width) {
                    // Split the left (src) range in two.
                    connectPortInstances(src.head(dst.width), dst);
                    src = src.tail(dst.width);
                    if (!dstRanges.hasNext()) {
                        // Should not happen (checked by the validator).
                        reporter.reportWarning(connection, 
                                "Source is wider than the destination. Outputs will be lost.");
                        break;
                    }
                    dst = dstRanges.next();
                } else if (src.width < dst.width) {
                    // Split the right (dst) range in two.
                    connectPortInstances(src, dst.head(src.width));
                    dst = dst.tail(src.width);
                    if (!srcRanges.hasNext()) {
                        if (connection.isIterated()) {
                            srcRanges = leftPorts.iterator();
                        } else {
                            reporter.reportWarning(connection, 
                                    "Destination is wider than the source. Inputs will be missing.");
                            break;
                        }
                    }
                    src = srcRanges.next();
                }
            }
        }
    }
    
    /**
     * Given a list of port references, as found on either side of a connection,
     * return a list of the port instance ranges referenced. These may be multiports,
     * and may be ports of a contained bank (a port representing ports of the bank
     * members) so the returned list includes ranges of banks and channels.
     * 
     * If a given port reference has the form `interleaved(b.m)`, where `b` is
     * a bank and `m` is a multiport, then the corresponding range in the returned
     * list is marked interleaved.
     * 
     * For example, if `b` and `m` have width 2, without the interleaved keyword,
     * the returned range represents the sequence `[b0.m0, b0.m1, b1.m0, b1.m1]`.
     * With the interleaved marking, the returned range represents the sequence
     * `[b0.m0, b1.m0, b0.m1, b1.m1]`. Both ranges will have width 4.
     * 
     * @param references The variable references on one side of the connection.
     * @param connection The connection.
     */
    private List<Range<PortInstance>> listPortInstances(
            List<VarRef> references, Connection connection
    ) {
        List<Range<PortInstance>> result = new ArrayList<Range<PortInstance>>();
        for (VarRef portRef : references) {
            // Simple error checking first.
            if (!(portRef.getVariable() instanceof Port)) {
                reporter.reportError(portRef, "Not a port.");
                return result;
            }
            // First, figure out which reactor we are dealing with.
            // The reactor we want is the container of the port.
            // If the port reference has no container, then the reactor is this one.
            var reactor = this;
            if (portRef.getContainer() != null) {
                reactor = getChildReactorInstance(portRef.getContainer());
            }
            // The reactor can be null only if there is an error in the code.
            // Skip this portRef so that diagram synthesis can complete.
            if (reactor != null) {
                PortInstance portInstance = reactor.lookupPortInstance((Port) portRef.getVariable());
                
                Range<PortInstance> range = new Range.Port(portInstance, connection);
                if (portRef.isInterleaved()) {
                    // Toggle interleaving at the depth of the reactor
                    // that is the parent of the port.
                    // FIXME: Here, we are assuming that the interleaved()
                    // keyword is only allowed on the multiports contained by
                    // contained reactors.
                    range = range.toggleInterleaved(portInstance.parent);
                }
                result.add(range);
            }
        }
        return result;
    }

    /**
     * If this is a bank of reactors, set the width.
     * It will be set to -1 if it cannot
     * be determined.
     */
    private void setInitialWidth() {
        WidthSpec widthSpec = definition.getWidthSpec();
        if (widthSpec != null) {
            // We need the instantiations list of the containing reactor,
            // not this one.
            width = ASTUtils.width(widthSpec, parent.instantiations());
        }
    }
    
    //////////////////////////////////////////////////////
    //// Private fields.

    /**
     * Cached reaction graph containing reactions that form a causality loop.
     */
    private ReactionInstanceGraph cachedReactionLoopGraph = null;
    
    /** 
     * One plus the number of contained reactor instances
     * (if a bank, in a bank element).
     */
    private int numReactorInstances = 1;
    
    /**
     * The offset relative to the parent. This is the sum of
     * the total number of reactor instances of peer reactors
     * (those with the same parent) that are instantiated before
     * this in the parent.
     */
    private int indexOffset = 0;
    
    /**
     * Cached version of the total number of reactions within
     * this reactor and its contained reactors.
     */
    private int totalNumberOfReactionsCache = -1;
}
