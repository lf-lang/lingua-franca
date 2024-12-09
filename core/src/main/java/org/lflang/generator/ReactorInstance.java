/*************
 * Copyright (c) 2019-2022, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator;

import static org.lflang.AttributeUtils.isEnclave;
import static org.lflang.ast.ASTUtils.getLiteralTimeValue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.TriggerInstance.BuiltinTriggerVariable;
import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Action;
import org.lflang.lf.Assignment;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Initializer;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Mode;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;
import org.lflang.lf.Timer;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;
import org.lflang.lf.WidthSpec;

/**
 * Representation of a compile-time instance of a reactor. If the reactor is instantiated as a bank
 * of reactors, or if any of its parents is instantiated as a bank of reactors, then one instance of
 * this ReactorInstance class represents all the runtime instances within these banks. The {@link
 * #getTotalWidth()} method returns the number of such runtime instances, which is the product of
 * the bank width of this reactor instance and the bank widths of all of its parents. There is
 * exactly one instance of this ReactorInstance class for each graphical rendition of a reactor in
 * the diagram view.
 *
 * <p>For the main reactor, which has no parent, once constructed, this object represents the entire
 * Lingua Franca program. If the program has causality loops (a programming error), then {@link
 * #hasCycles()} will return true and {@link #getCycles()} will return the ports and reaction
 * instances involved in the cycles.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 */
public class ReactorInstance extends NamedInstance<Instantiation> {

  /**
   * Create a new instantiation hierarchy that starts with the given top-level reactor.
   *
   * @param reactor The top-level reactor.
   * @param reporter The error reporter.
   */
  public ReactorInstance(Reactor reactor, MessageReporter reporter, List<Reactor> reactors) {
    this(ASTUtils.createInstantiation(reactor), null, reporter, -1, reactors);
    assert !reactors.isEmpty();
  }

  /**
   * Create a new instantiation hierarchy that starts with the given top-level reactor.
   *
   * @param reactor The top-level reactor.
   * @param reporter The error reporter.
   */
  public ReactorInstance(Reactor reactor, MessageReporter reporter) {
    this(ASTUtils.createInstantiation(reactor), null, reporter, -1, List.of());
  }

  /**
   * Create a new instantiation hierarchy that starts with the given top-level reactor but only
   * creates contained reactors up to the specified depth.
   *
   * @param reactor The top-level reactor.
   * @param reporter The error reporter.
   * @param desiredDepth The depth to which to go, or -1 to construct the full hierarchy.
   */
  public ReactorInstance(Reactor reactor, MessageReporter reporter, int desiredDepth) {
    this(ASTUtils.createInstantiation(reactor), null, reporter, desiredDepth, List.of());
  }

  /**
   * Create a new instantiation with the specified parent. This constructor is here to allow for
   * unit tests. It should not be used for any other purpose.
   *
   * @param reactor The top-level reactor.
   * @param parent The parent reactor instance.
   * @param reporter The error reporter.
   */
  public ReactorInstance(Reactor reactor, ReactorInstance parent, MessageReporter reporter) {
    this(ASTUtils.createInstantiation(reactor), parent, reporter, -1, List.of());
  }

  //////////////////////////////////////////////////////
  //// Public fields.

  /** The action instances belonging to this reactor instance. */
  public final List<ActionInstance> actions = new ArrayList<>();

  /**
   * The contained reactor instances, in order of declaration. For banks of reactors, this includes
   * both the bank definition Reactor (which has bankIndex == -2) followed by each of the bank
   * members (which have bankIndex >= 0).
   */
  public final List<ReactorInstance> children = new ArrayList<>();

  /** The input port instances belonging to this reactor instance. */
  public final List<PortInstance> inputs = new ArrayList<>();

  /** The output port instances belonging to this reactor instance. */
  public final List<PortInstance> outputs = new ArrayList<>();

  /** The state variable instances belonging to this reactor instance. */
  public final List<StateVariableInstance> states = new ArrayList<>();

  /** The parameters of this instance. */
  public final List<ParameterInstance> parameters = new ArrayList<>();

  /** List of reaction instances for this reactor instance. */
  public final List<ReactionInstance> reactions = new ArrayList<>();

  /** List of watchdog instances for this reactor instance. */
  public final List<WatchdogInstance> watchdogs = new ArrayList<>();

  /** The timer instances belonging to this reactor instance. */
  public final List<TimerInstance> timers = new ArrayList<>();

  /** The mode instances belonging to this reactor instance. */
  public final List<ModeInstance> modes = new ArrayList<>();

  /** The reactor declaration in the AST. This is either an import or Reactor declaration. */
  public final ReactorDecl reactorDeclaration;

  /** The reactor after imports are resolve. */
  public final Reactor reactorDefinition;

  /** Indicator that this reactor has itself as a parent, an error condition. */
  public final boolean recursive;

  // An enclave object if this ReactorInstance is an enclave. null if not
  public EnclaveInfo enclaveInfo = null;
  public TypeParameterizedReactor tpr;

  /**
   * The TPO level with which {@code this} was annotated, or {@code null} if there is no TPO
   * annotation. TPO is total port order. See
   * https://github.com/icyphy/lf-pubs/blob/54af48a97cc95058dbfb3333b427efb70294f66c/federated/TOMACS/paper.tex#L1353
   */
  public final Integer tpoLevel;

  //////////////////////////////////////////////////////
  //// Public methods.

  /**
   * Assign levels to all reactions within the same root as this reactor. The level of a reaction r
   * is equal to the length of the longest chain of reactions that must have the opportunity to
   * execute before r at each tag. This returns a non-empty graph if a causality cycle exists.
   *
   * <p>This method uses a variant of Kahn's algorithm, which is linear in V + E, where V is the
   * number of vertices (reactions) and E is the number of edges (dependencies between reactions).
   *
   * @return An empty graph if successful and otherwise a graph with runtime reaction instances that
   *     form cycles.
   */
  public ReactionInstanceGraph assignLevels() {
    if (depth != 0) return root().assignLevels();
    if (cachedReactionLoopGraph == null) {
      cachedReactionLoopGraph = new ReactionInstanceGraph(this);
    }
    return cachedReactionLoopGraph;
  }

  /**
   * Return the instance of a child rector created by the specified definition or null if there is
   * none.
   *
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
   * Clear any cached data in this reactor and its children. This is useful if a mutation has been
   * realized.
   */
  public void clearCaches() {
    clearCaches(true);
  }

  /**
   * Clear any cached data in this reactor and its children. This is useful if a mutation has been
   * realized.
   *
   * @param includingRuntimes If false, leave the runtime instances of reactions intact. This is
   *     useful for federated execution where levels are computed using the top-level connections,
   *     but then those connections are discarded.
   */
  public void clearCaches(boolean includingRuntimes) {
    if (includingRuntimes) cachedReactionLoopGraph = null;
    for (ReactorInstance child : children) {
      child.clearCaches(includingRuntimes);
    }
    for (PortInstance port : inputs) {
      port.clearCaches();
    }
    for (PortInstance port : outputs) {
      port.clearCaches();
    }
    for (ReactionInstance reaction : reactions) {
      reaction.clearCaches(includingRuntimes);
    }
    cachedCycles = null;
  }

  /**
   * Return the set of ReactionInstance and PortInstance that form causality loops in the topmost
   * parent reactor in the instantiation hierarchy. This will return an empty set if there are no
   * causality loops.
   */
  public Set<NamedInstance<?>> getCycles() {
    if (depth != 0) return root().getCycles();
    if (cachedCycles != null) return cachedCycles;
    cachedCycles = new LinkedHashSet<>();

    ReactionInstanceGraph reactionRuntimes = assignLevels();
    if (reactionRuntimes.nodes().size() > 0) {
      Set<ReactionInstance> reactions = new LinkedHashSet<>();
      Set<PortInstance> ports = new LinkedHashSet<>();
      // There are cycles. But the nodes set includes not
      // just the cycles, but also nodes that are downstream of the
      // cycles.  Use Tarjan's algorithm to get just the cycles.
      var cycleNodes = reactionRuntimes.getCycles();
      for (var cycle : cycleNodes) {
        for (ReactionInstance.Runtime runtime : cycle) {
          reactions.add(runtime.getReaction());
        }
      }
      // Need to figure out which ports are involved in the cycles.
      // It may not be all ports that depend on this reaction.
      for (ReactionInstance r : reactions) {
        for (TriggerInstance<? extends Variable> p : r.effects) {
          if (p instanceof PortInstance) {
            findPaths((PortInstance) p, reactions, ports);
          }
        }
      }
      cachedCycles.addAll(reactions);
      cachedCycles.addAll(ports);
    }

    return cachedCycles;
  }

  /**
   * Return the specified input by name or null if there is no such input.
   *
   * @param name The input name.
   */
  public PortInstance getInput(String name) {
    for (PortInstance port : inputs) {
      if (port.getName().equals(name)) {
        return port;
      }
    }
    return null;
  }

  /**
   * Override the base class to append [i_d], where d is the depth, if this reactor is in a bank of
   * reactors.
   *
   * @return The name of this instance.
   */
  @Override
  public String getName() {
    return this.definition.getName();
  }

  /**
   * @see NamedInstance#uniqueID()
   *     <p>Append {@code _main} to the name of the main reactor to allow instantiations within that
   *     reactor to have the same name.
   */
  @Override
  public String uniqueID() {
    if (this.isMainOrFederated()) {
      if (reactorDefinition.isFederated() && !super.uniqueID().startsWith("federate__"))
        return "federate__" + super.uniqueID() + "_main";
      return super.uniqueID() + "_main";
    }
    return super.uniqueID();
  }

  /**
   * Return the specified output by name or null if there is no such output.
   *
   * @param name The output name.
   */
  public PortInstance getOutput(String name) {
    for (PortInstance port : outputs) {
      if (port.getName().equals(name)) {
        return port;
      }
    }
    return null;
  }

  /**
   * Return a parameter matching the specified name if the reactor has one and otherwise return
   * null.
   *
   * @param name The parameter name.
   */
  public ParameterInstance getParameter(String name) {
    for (ParameterInstance parameter : parameters) {
      if (parameter.getName().equals(name)) {
        return parameter;
      }
    }
    return null;
  }

  /** Return the startup trigger or null if not used in any reaction. */
  public TriggerInstance<BuiltinTriggerVariable> getStartupTrigger() {
    return builtinTriggers.get(BuiltinTrigger.STARTUP);
  }

  /** Return the shutdown trigger or null if not used in any reaction. */
  public TriggerInstance<BuiltinTriggerVariable> getShutdownTrigger() {
    return builtinTriggers.get(BuiltinTrigger.SHUTDOWN);
  }

  /**
   * If this reactor is a bank or any of its parents is a bank, return the total number of runtime
   * instances, which is the product of the widths of all the parents. Return -1 if the width cannot
   * be determined.
   */
  public int getTotalWidth() {
    return getTotalWidth(0);
  }

  /**
   * If this reactor is a bank or any of its parents is a bank, return the total number of runtime
   * instances, which is the product of the widths of all the parents. Return -1 if the width cannot
   * be determined.
   *
   * @param atDepth The depth at which to determine the width. Use 0 to get the total number of
   *     instances. Use 1 to get the number of instances within a single top-level bank member (this
   *     is useful for federates).
   */
  public int getTotalWidth(int atDepth) {
    if (width <= 0) return -1;
    if (depth <= atDepth) return 1;
    int result = width;
    ReactorInstance p = parent;
    while (p != null && p.depth > atDepth) {
      if (p.width <= 0) return -1;
      result *= p.width;
      p = p.parent;
    }
    return result;
  }

  /**
   * Return the trigger instances (input ports, timers, and actions that trigger reactions)
   * belonging to this reactor instance.
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
   * Return the trigger instances (input ports, timers, and actions that trigger reactions) together
   * the ports that the reaction reads but that don't trigger it.
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

  /** Return true if the top-level parent of this reactor has causality cycles. */
  public boolean hasCycles() {
    return assignLevels().nodeCount() != 0;
  }

  /**
   * Given a parameter definition for this reactor, return the initial integer value of the
   * parameter. If the parameter is overridden when instantiating this reactor or any of its
   * containing reactors, use that value. Otherwise, use the default value in the reactor
   * definition. If the parameter cannot be found or its value is not an integer, return null.
   *
   * @param parameter The parameter definition (a syntactic object in the AST).
   * @return An integer value or null.
   */
  public Integer initialIntParameterValue(Parameter parameter) {
    return ASTUtils.initialValueInt(parameter, instantiations());
  }

  public Expression resolveParameters(Expression e) {
    return LfExpressionVisitor.dispatch(e, this, ParameterInliner.INSTANCE);
  }

  private static final class ParameterInliner
      extends LfExpressionVisitor.LfExpressionDeepCopyVisitor<ReactorInstance> {
    static final ParameterInliner INSTANCE = new ParameterInliner();

    @Override
    public Expression visitParameterRef(ParameterReference expr, ReactorInstance instance) {
      if (!ASTUtils.belongsTo(expr.getParameter(), instance.definition)) {
        throw new IllegalArgumentException(
            "Parameter "
                + expr.getParameter().getName()
                + " is not a parameter of reactor instance "
                + instance.getName()
                + ".");
      }

      Optional<Assignment> assignment =
          instance.definition.getParameters().stream()
              .filter(it -> it.getLhs().equals(expr.getParameter()))
              .findAny(); // There is at most one

      if (assignment.isPresent()) {
        // replace the parameter with its value.
        Expression value = assignment.get().getRhs().getExpr();
        // recursively resolve parameters
        return instance.getParent().resolveParameters(value);
      } else {
        // In that case use the default value. Default values
        // cannot use parameter values, so they don't need to
        // be recursively resolved.
        Initializer init = expr.getParameter().getInit();
        Expression defaultValue = init.getExpr();
        if (defaultValue == null) {
          // this is a problem
          return super.visitParameterRef(expr, instance);
        }
        return defaultValue;
      }
    }
  }

  /**
   * Return a list of Instantiation objects for evaluating parameter values. The first object in the
   * list is the AST Instantiation that created this reactor instance, the second is the AST
   * instantiation that created the containing reactor instance, and so on until there are no more
   * containing reactor instances. This will return an empty list if this reactor instance is at the
   * top level (is main).
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
   *
   * @return true if a reactor is a bank, false otherwise
   */
  public boolean isBank() {
    return definition.getWidthSpec() != null;
  }

  /**
   * Returns whether this is a main or federated reactor.
   *
   * @return true if reactor definition is marked as main or federated, false otherwise.
   */
  public boolean isMainOrFederated() {
    return reactorDefinition != null
        && (reactorDefinition.isMain() || reactorDefinition.isFederated());
  }

  /**
   * Return true if the specified reactor instance is either equal to this reactor instance or a
   * parent of it.
   *
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
   * Return the action instance within this reactor instance corresponding to the specified action
   * reference.
   *
   * @param action The action as an AST node.
   * @return The corresponding action instance or null if the action does not belong to this
   *     reactor.
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
   * Given a parameter definition, return the parameter instance corresponding to that definition,
   * or null if there is no such instance.
   *
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
   * Given a port definition, return the port instance corresponding to that definition, or null if
   * there is no such instance.
   *
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
   * Given a reference to a port belonging to this reactor instance, return the port instance.
   * Return null if there is no such instance.
   *
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
   * Return the reaction instance within this reactor instance corresponding to the specified
   * reaction.
   *
   * @param reaction The reaction as an AST node.
   * @return The corresponding reaction instance or null if the reaction does not belong to this
   *     reactor.
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
   * Return the reactor instance within this reactor that has the specified instantiation. Note that
   * this may be a bank of reactors. Return null if there is no such reactor instance.
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
   * Return the timer instance within this reactor instance corresponding to the specified timer
   * reference.
   *
   * @param timer The timer as an AST node.
   * @return The corresponding timer instance or null if the timer does not belong to this reactor.
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
   * Returns the mode instance within this reactor instance corresponding to the specified mode
   * reference.
   *
   * @param mode The mode as an AST node.
   * @return The corresponding mode instance or null if the mode does not belong to this reactor.
   */
  public ModeInstance lookupModeInstance(Mode mode) {
    for (ModeInstance modeInstance : modes) {
      if (modeInstance.definition == mode) {
        return modeInstance;
      }
    }
    return null;
  }

  /**
   * Return the watchdog instance within this reactor instance corresponding to the specified
   * watchdog reference.
   *
   * @param watchdog The watchdog as an AST node.
   * @return The corresponding watchdog instance or null if the watchdog does not belong to this
   *     reactor.
   */
  public WatchdogInstance lookupWatchdogInstance(Watchdog watchdog) {
    for (WatchdogInstance watchdogInstance : watchdogs) {
      if (watchdogInstance.getDefinition() == watchdog) {
        return watchdogInstance;
      }
    }
    return null;
  }

  /** Return a descriptive string. */
  @Override
  public String toString() {
    return "ReactorInstance " + getFullName();
  }

  /**
   * Assuming that the given expression denotes a valid time, return a time value.
   *
   * <p>If the value is given as a parameter reference, this will look up the precise time value
   * assigned to this reactor instance.
   */
  public TimeValue getTimeValue(Expression expr) {
    Expression resolved = resolveParameters(expr);
    return getLiteralTimeValue(resolved);
  }

  //////////////////////////////////////////////////////
  //// Protected fields.

  /** The generator that created this reactor instance. */
  protected MessageReporter reporter; // FIXME: This accumulates a lot of redundant references

  /** The map of used built-in triggers. */
  protected Map<BuiltinTrigger, TriggerInstance<BuiltinTriggerVariable>> builtinTriggers =
      new HashMap<>();

  /** The nested list of instantiations that created this reactor instance. */
  protected List<Instantiation> _instantiations;

  //////////////////////////////////////////////////////
  //// Protected methods.

  /**
   * Create all the reaction instances of this reactor instance and record the dependencies and
   * antidependencies between ports, actions, and timers and reactions. This also records the
   * dependencies between reactions that follows from the order in which they are defined.
   */
  protected void createReactionInstances() {
    List<Reaction> reactions = ASTUtils.allReactions(reactorDefinition);
    if (reactions != null) {
      int count = 0;

      // Check for startup and shutdown triggers.
      for (Reaction reaction : reactions) {
        // Create the reaction instance.
        var reactionInstance = new ReactionInstance(reaction, this, count++);
        // Add the reaction instance to the map of reactions for this
        // reactor.
        this.reactions.add(reactionInstance);
      }
    }
  }

  /** Returns the built-in trigger or create a new one if none exists. */
  protected TriggerInstance<BuiltinTriggerVariable> getOrCreateBuiltinTrigger(
      BuiltinTriggerRef trigger) {
    return builtinTriggers.computeIfAbsent(
        trigger.getType(), ref -> TriggerInstance.builtinTrigger(trigger, this));
  }

  /** Create all the watchdog instances of this reactor instance. */
  protected void createWatchdogInstances() {
    List<Watchdog> watchdogs = ASTUtils.allWatchdogs(reactorDefinition);
    if (watchdogs != null) {
      for (Watchdog watchdog : watchdogs) {
        // Create the watchdog instance.
        var watchdogInstance = new WatchdogInstance(watchdog, this);

        // Add the watchdog instance to the list of watchdogs for this
        // reactor.
        this.watchdogs.add(watchdogInstance);
      }
    }
  }

  ////////////////////////////////////////
  //// Private constructors

  /**
   * Create a runtime instance from the specified definition and with the specified parent that
   * instantiated it.
   *
   * @param definition The instantiation statement in the AST.
   * @param parent The parent, or null for the main rector.
   * @param reporter An error reporter.
   * @param desiredDepth The depth to which to expand the hierarchy.
   */
  public ReactorInstance(
      Instantiation definition,
      ReactorInstance parent,
      MessageReporter reporter,
      int desiredDepth,
      List<Reactor> reactors) {
    super(definition, parent);
    this.tpoLevel =
        definition.getAttributes().stream()
            .filter(it -> it.getAttrName().equals("_tpoLevel"))
            .map(it -> it.getAttrParms().stream().findAny().orElseThrow())
            .map(it -> Integer.parseInt(it.getValue()))
            .findFirst()
            .orElse(null);
    this.reporter = reporter;
    this.reactorDeclaration = definition.getReactorClass();
    this.reactorDefinition = ASTUtils.toDefinition(reactorDeclaration);
    this.tpr =
        parent == null
            ? new TypeParameterizedReactor(definition, reactors)
            : new TypeParameterizedReactor(definition, parent.tpr);

    // If this instance is an enclave (or the main reactor). Create an
    // enclaveInfo object to track information about the enclave needed for
    // later code-generation
    if (isEnclave(definition) || this.isMainOrFederated()) {
      enclaveInfo = new EnclaveInfo(this);
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
    } while (currentParent != null);

    this.recursive = foundSelfAsParent;
    if (recursive) {
      reporter.at(definition).error("Recursive reactor instantiation.");
    }

    // If the reactor definition is null, give up here. Otherwise, diagram generation
    // will fail an NPE.
    if (reactorDefinition == null) {
      reporter.at(definition).error("Reactor instantiation has no matching reactor definition.");
      return;
    }

    setInitialWidth();

    // Apply overrides and instantiate parameters for this reactor instance.
    for (Parameter parameter : ASTUtils.allParameters(reactorDefinition)) {
      this.parameters.add(new ParameterInstance(parameter, this));
    }

    // Instantiate inputs for this reactor instance.
    for (Input inputDecl : ASTUtils.allInputs(reactorDefinition)) {
      this.inputs.add(new PortInstance(inputDecl, this, reporter));
    }

    // Instantiate outputs for this reactor instance.
    for (Output outputDecl : ASTUtils.allOutputs(reactorDefinition)) {
      this.outputs.add(new PortInstance(outputDecl, this, reporter));
    }

    // Instantiate state variables for this reactor instance.
    for (StateVar state : ASTUtils.allStateVars(reactorDefinition)) {
      this.states.add(new StateVariableInstance(state, this, reporter));
    }

    // Do not process content (except interface above) if recursive.
    if (!recursive && (desiredDepth < 0 || this.depth < desiredDepth)) {
      // Instantiate children for this reactor instance.
      // While doing this, assign an index offset to each.
      for (Instantiation child : ASTUtils.allInstantiations(reactorDefinition)) {
        var childInstance = new ReactorInstance(child, this, reporter, desiredDepth, reactors);
        this.children.add(childInstance);
      }

      // Instantiate timers for this reactor instance
      for (Timer timerDecl : ASTUtils.allTimers(reactorDefinition)) {
        this.timers.add(new TimerInstance(timerDecl, this));
      }

      // Instantiate actions for this reactor instance
      for (Action actionDecl : ASTUtils.allActions(reactorDefinition)) {
        this.actions.add(new ActionInstance(actionDecl, this));
      }

      createWatchdogInstances();

      establishPortConnections();

      // Create the reaction instances in this reactor instance.
      // This also establishes all the implied dependencies.
      // Note that this can only happen _after_ the children,
      // port, action, and timer instances have been created.
      createReactionInstances();

      // Instantiate modes for this reactor instance
      // This must come after the child elements (reactions, etc) of this reactor
      // are created in order to allow their association with modes
      for (Mode modeDecl : ASTUtils.allModes(reactorDefinition)) {
        this.modes.add(new ModeInstance(modeDecl, this));
      }
      for (ModeInstance mode : this.modes) {
        mode.setupTranstions();
      }
    }
  }

  //////////////////////////////////////////////////////
  //// Private methods.

  /**
   * Connect the given left port range to the given right port range.
   *
   * <p>NOTE: This method is public to enable its use in unit tests. Otherwise, it should be
   * private. This is why it is defined here, in the section labeled "Private methods."
   *
   * @param src The source range.
   * @param dst The destination range.
   * @param connection The connection establishing this relationship.
   */
  public static void connectPortInstances(
      RuntimeRange<PortInstance> src, RuntimeRange<PortInstance> dst, Connection connection) {
    SendRange range = new SendRange(src, dst, src._interleaved, connection);
    src.instance.dependentPorts.add(range);
    dst.instance.dependsOnPorts.add(src);
  }

  /**
   * Populate connectivity information in the port instances. Note that this can only happen _after_
   * the children and port instances have been created. Unfortunately, we have to do some
   * complicated things here to support multiport-to-multiport, multiport-to-bank, and
   * bank-to-multiport communication. The principle being followed is: in each connection statement,
   * for each port instance on the left, connect to the next available port on the right.
   */
  private void establishPortConnections() {
    for (Connection connection : ASTUtils.allConnections(reactorDefinition)) {
      List<RuntimeRange<PortInstance>> leftPorts =
          listPortInstances(connection.getLeftPorts(), connection);
      Iterator<RuntimeRange<PortInstance>> srcRanges = leftPorts.iterator();
      List<RuntimeRange<PortInstance>> rightPorts =
          listPortInstances(connection.getRightPorts(), connection);
      Iterator<RuntimeRange<PortInstance>> dstRanges = rightPorts.iterator();

      // Check for empty lists.
      if (!srcRanges.hasNext()) {
        if (dstRanges.hasNext()) {
          reporter.at(connection).warning("No sources to provide inputs.");
        }
        return;
      } else if (!dstRanges.hasNext()) {
        reporter.at(connection).warning("No destination. Outputs will be lost.");
        return;
      }

      RuntimeRange<PortInstance> src = srcRanges.next();
      RuntimeRange<PortInstance> dst = dstRanges.next();

      while (true) {
        if (dst.width <= -1 || src.width <= -1) {
          // The width on one side or the other is not known.  Make all possible connections.
          connectPortInstances(src, dst, connection);
          if (dstRanges.hasNext()) {
            dst = dstRanges.next();
          } else if (srcRanges.hasNext()) {
            src = srcRanges.next();
          } else {
            break;
          }
        } else if (dst.width == src.width) {
          connectPortInstances(src, dst, connection);
          if (!dstRanges.hasNext()) {
            if (srcRanges.hasNext()) {
              // Should not happen (checked by the validator).
              reporter
                  .at(connection)
                  .warning("Source is wider than the destination. Outputs will be lost.");
            }
            break;
          }
          if (!srcRanges.hasNext()) {
            if (connection.isIterated()) {
              srcRanges = leftPorts.iterator();
            } else {
              if (dstRanges.hasNext()) {
                // Should not happen (checked by the validator).
                reporter
                    .at(connection)
                    .warning("Destination is wider than the source. Inputs will be missing.");
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
            reporter
                .at(connection)
                .warning("Source is wider than the destination. Outputs will be lost.");
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
              reporter
                  .at(connection)
                  .warning("Destination is wider than the source. Inputs will be missing.");
              break;
            }
          }
          src = srcRanges.next();
        }
      }
    }
  }

  /**
   * If path exists from the specified port to any reaction in the specified set of reactions, then
   * add the specified port and all ports along the path to the specified set of ports.
   *
   * @return True if the specified port was added.
   */
  private boolean findPaths(
      PortInstance port, Set<ReactionInstance> reactions, Set<PortInstance> ports) {
    if (ports.contains(port)) return false;
    boolean result = false;
    for (ReactionInstance d : port.getDependentReactions()) {
      if (reactions.contains(d)) ports.add(port);
      result = true;
    }
    // Perform a depth-first search.
    for (SendRange r : port.dependentPorts) {
      for (RuntimeRange<PortInstance> p : r.destinations) {
        boolean added = findPaths(p.instance, reactions, ports);
        if (added) {
          result = true;
          ports.add(port);
        }
      }
    }
    return result;
  }

  /**
   * Given a list of port references, as found on either side of a connection, return a list of the
   * port instance ranges referenced. These may be multiports, and may be ports of a contained bank
   * (a port representing ports of the bank members) so the returned list includes ranges of banks
   * and channels.
   *
   * <p>If a given port reference has the form {@code interleaved(b.m)}, where {@code b} is a bank
   * and {@code m} is a multiport, then the corresponding range in the returned list is marked
   * interleaved.
   *
   * <p>For example, if {@code b} and {@code m} have width 2, without the interleaved keyword, the
   * returned range represents the sequence {@code [b0.m0, b0.m1, b1.m0, b1.m1]}. With the
   * interleaved marking, the returned range represents the sequence {@code [b0.m0, b1.m0, b0.m1,
   * b1.m1]}. Both ranges will have width 4.
   *
   * @param references The variable references on one side of the connection.
   * @param connection The connection.
   */
  private List<RuntimeRange<PortInstance>> listPortInstances(
      List<VarRef> references, Connection connection) {
    List<RuntimeRange<PortInstance>> result = new ArrayList<>();
    List<RuntimeRange<PortInstance>> tails = new LinkedList<>();
    int count = 0;
    for (VarRef portRef : references) {
      // Simple error checking first.
      if (!(portRef.getVariable() instanceof Port)) {
        reporter.at(portRef).error("Not a port.");
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

        Set<ReactorInstance> interleaved = new LinkedHashSet<>();
        if (portRef.isInterleaved()) {
          // NOTE: Here, we are assuming that the interleaved()
          // keyword is only allowed on the multiports contained by
          // contained reactors.
          interleaved.add(portInstance.parent);
        }
        RuntimeRange<PortInstance> range = new RuntimeRange.Port(portInstance, interleaved);
        // If this portRef is not the last one in the references list
        // then we have to check whether the range can be incremented at
        // the lowest two levels (port and container).  If not,
        // split the range and add the tail to list to iterate over again.
        // The reason for this is that the connection has only local visibility,
        // but the range width may be reflective of bank structure higher
        // in the hierarchy.
        if (count < references.size() - 1) {
          int portWidth = portInstance.width;
          int portParentWidth = portInstance.parent.width;
          // If the port is being connected on the inside and there is
          // more than one port in the list, then we can only connect one
          // bank member at a time.
          if (reactor == this && references.size() > 1) {
            portParentWidth = 1;
          }
          int widthBound = portWidth * portParentWidth;

          // If either of these widths cannot be determined, assume infinite.
          if (portWidth < 0) widthBound = Integer.MAX_VALUE;
          if (portParentWidth < 0) widthBound = Integer.MAX_VALUE;

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
    while (tails.size() > 0) {
      List<RuntimeRange<PortInstance>> moreTails = new LinkedList<>();
      count = 0;
      for (RuntimeRange<PortInstance> tail : tails) {
        if (count < tails.size() - 1) {
          int widthBound = tail.instance.width;
          if (tail._interleaved.contains(tail.instance.parent)) {
            widthBound = tail.instance.parent.width;
          }
          // If the width cannot be determined, assume infinite.
          if (widthBound < 0) widthBound = Integer.MAX_VALUE;

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
   * If this is a bank of reactors, set the width. It will be set to -1 if it cannot be determined.
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

  /** Cached set of reactions and ports that form a causality loop. */
  private Set<NamedInstance<?>> cachedCycles;

  /** Cached reaction graph containing reactions that form a causality loop. */
  private ReactionInstanceGraph cachedReactionLoopGraph = null;

  /**
   * Return true if this is a generated delay reactor that originates from an "after" delay on a
   * connection.
   *
   * @return True if this is a generated delay, false otherwise.
   */
  public boolean isGeneratedDelay() {
    // FIXME: hacky string matching again...
    return this.definition
        .getReactorClass()
        .getName()
        .contains(DelayBodyGenerator.GEN_DELAY_CLASS_NAME);
  }
}
