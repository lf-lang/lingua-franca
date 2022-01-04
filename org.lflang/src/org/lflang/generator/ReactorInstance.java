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
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.JavaAstUtils;
import org.lflang.TimeValue;
import org.lflang.generator.TriggerInstance.BuiltinTriggerVariable;
import org.lflang.lf.Action;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
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
     * @param parent The parent reactor instance.
     * @param reporter The error reporter.
     */
    public ReactorInstance(Reactor reactor, ReactorInstance parent, ErrorReporter reporter) {
        this(ASTUtils.createInstantiation(reactor), parent, reporter, -1, null);
    }

    //////////////////////////////////////////////////////
    //// Public fields.

    /** The action instances belonging to this reactor instance. */
    public List<ActionInstance> actions = new ArrayList<>();

    /**
     * The contained reactor instances, in order of declaration.
     * For banks of reactors, this includes both the bank definition
     * Reactor (which has bankIndex == -2) followed by each of the
     * bank members (which have bankIndex >= 0).
     */
    public final List<ReactorInstance> children = new ArrayList<>();
    
    /**
     * The ID of this reactor instance. This is 0 for a top-level (main) reactor
     * and increases for each created reactor in the order created until it
     * reaches main's {@link #totalNumberOfChildren()}.
     */
    public final int id;

    /** The input port instances belonging to this reactor instance. */
    public final List<PortInstance> inputs = new ArrayList<>();

    /** The output port instances belonging to this reactor instance. */
    public final List<PortInstance> outputs = new ArrayList<>();

    /** The parameters of this instance. */
    public final List<ParameterInstance> parameters = new ArrayList<>();

    /** List of reaction instances for this reactor instance. */
    public final List<ReactionInstance> reactions = new ArrayList<>();

    /** The timer instances belonging to this reactor instance. */
    public final List<TimerInstance> timers = new ArrayList<>();

    /** The reactor declaration in the AST. This is either an import or Reactor declaration. */
    public final ReactorDecl reactorDeclaration;

    /** The reactor after imports are resolve. */
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
     * Clear any cached data in this reactor and its children.
     * This is useful if a mutation has been realized.
     */
    public void clearCaches() {
        cachedReactionLoopGraph = null;
        totalNumChildrenCache = -1;
        for (ReactorInstance child : children) {
            child.clearCaches();
        }
        for (PortInstance port : inputs) {
            port.clearCaches();
        }
        for (PortInstance port : outputs) {
            port.clearCaches();
        }
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
        // FIXME FIXME get rid of this.
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
        // FIXME FIXME get rid of this.
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
        // FIXME FIXME: get rid of this.
        if (width < 0 || numReactorInstances < 0) return -1;
        return width * numReactorInstances;
    }
    
    /**
     * If this reactor is a bank or any of its parents is a bank,
     * return the total number of runtime instances, which is the product
     * of the widths of all the parents.
     * Return -1 if the width cannot be determined.
     */
    public int getTotalWidth() {
        if (width <= 0) return -1;
        if (depth > 0) {
            int parentWidth = parent.getTotalWidth();
            if (parentWidth <= 0) return -1;
            return (parentWidth * width);
        } else {
            return 1;
        }
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
            _instantiations = new ArrayList<>();
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
    
    /**
     * Return true if the specified reactor instance is either equal to this
     * reactor instance or a parent of it.
     * @param r The reactor instance.
     */
    public boolean isParent(ReactorInstance r) {
        ReactorInstance p = this;
        while (p != null) {
            if (p == r) return true;
            p = p.getParent();
        }
        return false;
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
     * Return the total number of children in this reactor
     * and all its contained reactors. Each bank counts as one child.
     */
    public int totalNumberOfChildren() {
        if (totalNumChildrenCache < 0) {
            totalNumChildrenCache = children.size();
            for (ReactorInstance containedReactor : children) {
                totalNumChildrenCache += containedReactor.totalNumberOfChildren();
            }
        }
        return totalNumChildrenCache;
    }

    /**
     * Assuming that the given value denotes a valid time, return a time value.
     *
     * If the value is given as a parameter reference, this will look up the
     * precise time value assigned to this reactor instance.
     */
    public TimeValue getTimeValue(Value v) {
        Parameter p = v.getParameter();
        if (p != null) {
            return JavaAstUtils.getLiteralTimeValue(lookupParameterInstance(p).getInitialValue().get(0));
        } else {
            return JavaAstUtils.getLiteralTimeValue(v);
        }
    }

    /**
     * Assuming that the given delay denotes a valid time, return a time value.
     *
     * If the delay is given as a parameter reference, this will look up the
     * precise time value assigned to this reactor instance.
     */
    public TimeValue getTimeValue(Delay d) {
        Parameter p = d.getParameter();
        if (p != null) {
            return JavaAstUtils.getLiteralTimeValue(lookupParameterInstance(p).getInitialValue().get(0));
        } else {
            return JavaAstUtils.toTimeValue(d.getTime());
        }
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
    protected Set<Reaction> unorderedReactions = new LinkedHashSet<>();

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
            startupTrigger = new TriggerInstance<>(
                TriggerInstance.BuiltinTrigger.STARTUP, trigger, this);
        }
        return startupTrigger;
    }
    
    /**
     * Returns the shutdown trigger or create a new one if none exists.
     */
    protected TriggerInstance<? extends Variable> getOrCreateShutdown(TriggerRef trigger) {
        if (shutdownTrigger == null) {
            shutdownTrigger = new TriggerInstance<>(
                TriggerInstance.BuiltinTrigger.SHUTDOWN, trigger, this);
        }
        return shutdownTrigger;
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
        this.reactorDeclaration = definition.getReactorClass();
        this.reactorDefinition = ASTUtils.toDefinition(reactorDeclaration);
        this.id = root().childCount++;
        
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
     * 
     * NOTE: This method is public to enable its use in unit tests.
     * Otherwise, it should be private. This is why it is defined here,
     * in the section labeled "Private methods."
     * 
     * @param src The source range.
     * @param dst The destination range.
     * @param connection The connection establishing this relationship.
     */
    public static void connectPortInstances(
            RuntimeRange<PortInstance> src,
            RuntimeRange<PortInstance> dst,
            Connection connection
    ) {
        SendRange range = new SendRange(src, dst, src._interleaved, connection);
        src.instance.dependentPorts.add(range);
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
            List<RuntimeRange<PortInstance>> leftPorts = listPortInstances(connection.getLeftPorts(), connection);
            Iterator<RuntimeRange<PortInstance>> srcRanges = leftPorts.iterator();
            List<RuntimeRange<PortInstance>> rightPorts = listPortInstances(connection.getRightPorts(), connection);
            Iterator<RuntimeRange<PortInstance>> dstRanges = rightPorts.iterator();
            
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
            
            RuntimeRange<PortInstance> src = srcRanges.next();
            RuntimeRange<PortInstance> dst = dstRanges.next();

            while(true) {
                if (dst.width == src.width) {
                    connectPortInstances(src, dst, connection);
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
                    connectPortInstances(src.head(dst.width), dst, connection);
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
                    connectPortInstances(src, dst.head(src.width), connection);
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
    private List<RuntimeRange<PortInstance>> listPortInstances(
            List<VarRef> references, Connection connection
    ) {
        List<RuntimeRange<PortInstance>> result = new ArrayList<RuntimeRange<PortInstance>>();
        List<RuntimeRange<PortInstance>> tails = new LinkedList<RuntimeRange<PortInstance>>();
        int count = 0;
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
                PortInstance portInstance = reactor.lookupPortInstance(
                        (Port) portRef.getVariable());
                
                Set<ReactorInstance> interleaved = new LinkedHashSet<ReactorInstance>();
                if (portRef.isInterleaved()) {
                    // NOTE: Here, we are assuming that the interleaved()
                    // keyword is only allowed on the multiports contained by
                    // contained reactors.
                    interleaved.add(portInstance.parent);
                }
                RuntimeRange<PortInstance> range = new RuntimeRange.Port(
                        portInstance, interleaved);
                // If this portRef is not the last one in the references list
                // then we have to check whether the range can be incremented at
                // the lowest two levels (port and container).  If not,
                // split the range and add the tail to list to iterate over again.
                // The reason for this is that the connection has only local visibility,
                // but the range width may be reflective of bank structure higher
                // in the hierarchy.
                if (count < references.size() - 1) {
                    int widthBound = portInstance.width * portInstance.parent.width;
                    if (widthBound < range.width) {
                        // Need to split the range.
                        tails.add(range.tail(widthBound));
                        range = range.head(widthBound);
                    }
                }
                result.add(range);
            }
        }
        // Iterate over the tails.
        while(tails.size() > 0) {
            List<RuntimeRange<PortInstance>> moreTails = new LinkedList<RuntimeRange<PortInstance>>();
            count = 0;
            for (RuntimeRange<PortInstance> tail : tails) {
                if (count < tails.size() - 1) {
                    int widthBound = tail.instance.width;
                    if (tail._interleaved.contains(tail.instance.parent)) {
                        widthBound = tail.instance.parent.width;
                    }
                    if (widthBound < tail.width) {
                        // Need to split the range again
                        moreTails.add(tail.tail(widthBound));
                        tail = tail.head(widthBound);
                    }
                }
                result.add(tail);
            }
            tails = moreTails;
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
     * Count of children created for assigning IDs. This should only be
     * used by a top-level reactor.
     */
    private int childCount = 0;
    
    /**
     * Cache of the deep number of children.
     */
    private int totalNumChildrenCache = -1;
    
    // FIXME FIXME: Get rid of the following.
    
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
}
