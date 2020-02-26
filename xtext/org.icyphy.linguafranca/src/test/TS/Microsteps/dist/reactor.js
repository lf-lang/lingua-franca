"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.App = exports.InPort = exports.OutPort = exports.Port = exports.Reactor = exports.Triggers = exports.Args = exports.Mutation = exports.Timer = exports.Scheduler = exports.State = exports.Parameter = exports.Shutdown = exports.Startup = exports.Action = exports.Reaction = void 0;

var _util = require("./util");

var _time = require("./time");

function _defineProperty(obj, key, value) { if (key in obj) { Object.defineProperty(obj, key, { value: value, enumerable: true, configurable: true, writable: true }); } else { obj[key] = value; } return obj; }

//Log.setGlobalLevel(Log.levels.DEBUG);
//---------------------------------------------------------------------//
// Modules                                                             //
//---------------------------------------------------------------------//

/**
 * Timer used for precisely timing the triggering of reactions.
 */
const NanoTimer = require('nanotimer'); //---------------------------------------------------------------------//
// Types                                        //
//---------------------------------------------------------------------//

/**
 * Type that denotes the absence of data exchanged between ports.
 */


//---------------------------------------------------------------------//
// Core Reactor Classes                                                //
//---------------------------------------------------------------------//
class Descendant {
  constructor(__parent__) {
    this.__parent__ = __parent__;

    _defineProperty(this, "alias", void 0);
  }
  /**
   * Return a string that identifies this component.
   * The name is a path constructed as TopLevelParentName/.../ParentName/ThisReactorsName
   */


  getFullyQualifiedName() {
    var path = "";

    if (this.__parent__ != null) {
      path = this.__parent__.getFullyQualifiedName();
    }

    if (path != "") {
      path += "/" + this.getName();
    } else {
      path = this.getName();
    }

    return path;
  }
  /* Return a globally unique identifier. */


  getName() {
    var count = 0;
    var suffix = "";

    if (this.alias) {
      return this.alias;
    } else if (this.__parent__) {
      for (const [key, value] of Object.entries(this.__parent__)) {
        if (value === this) {
          return `${key}`;
        } // Count instantiations of the same object among entries
        // in order to report unique names (within the scope of
        // the reactor) for each entry.


        if (value && this.constructor === value.constructor) {
          count++;
        }
      }
    }

    if (count > 0) {
      suffix = "(" + count + ")";
    }

    return this.constructor.name + suffix;
  }

  setAlias(name) {
    this.alias = name;
  }

}
/**
 * Generic base class for reactions. The type parameter `T` denotes the
 * type of the argument list that the function `react` is applied to when
 * this reaction gets triggered.
 */


class Reaction {
  /** Priority derived from this reaction's location in 
   *  the directed acyclic precedence graph. */
  // getID(): Reaction<unknown> {
  //     return this;
  // }
  getNext() {
    return this.next;
  }

  setNext(node) {
    this.next = node;
  }

  toString() {
    return this.__parent__.getFullyQualifiedName() + "[R" + this.__parent__.getReactionIndex(this) + "]";
  }

  setReact(r) {
    return this;
  }

  getDependencies() {
    var deps = new Set();
    var antideps = new Set();
    var vars = new Set();

    for (let a of this.args.tuple.concat(this.trigs.list)) {
      if (a instanceof Port) {
        // FIXME: handle Writers and hierarchical references!
        if (this.__parent__._isUpstream(a)) {
          deps.add(a);
        }

        if (this.__parent__._isDownstream(a)) {
          antideps.add(a);
        }
      } else if (a instanceof Writer) {
        if (this.__parent__._isDownstream(a.getPort())) {
          antideps.add(a.getPort());
        }
      } else {
        // Handle hierarchical references.
        for (let p of Object.getOwnPropertyNames(a)) {
          let prop = Object.getOwnPropertyDescriptor(a, p);

          if ((prop === null || prop === void 0 ? void 0 : prop.value) instanceof Port) {
            if (this.__parent__._isUpstream(prop.value)) {
              deps.add(prop.value);
            }

            if (this.__parent__._isDownstream(prop.value)) {
              antideps.add(prop.value);
            }
          }
        }
      }
    }

    return [deps, antideps];
  }

  getPriority() {
    return this.priority;
  }

  hasPriorityOver(node) {
    if (node != null && this.getPriority() < node.getPriority()) {
      return true;
    } else {
      return false;
    }
  }

  updateIfDuplicateOf(node) {
    return Object.is(this, node);
  } //A reaction defaults to not having a deadline  FIXME: we want the deadline to have access to the same variables


  /** 
   * Construct a new Reaction by passing in a reference to the reactor that contains it,
   * the variables that trigger it, and the arguments passed to the react function.
   * @param state state shared among reactions
   */
  constructor(__parent__, trigs, args, react, deadline, late = () => {
    _util.Log.global.warn("Deadline violation occurred!");
  }) {
    this.__parent__ = __parent__;
    this.trigs = trigs;
    this.args = args;
    this.react = react;
    this.deadline = deadline;
    this.late = late;

    _defineProperty(this, "priority", Number.MAX_SAFE_INTEGER);

    _defineProperty(this, "util", void 0);

    _defineProperty(this, "state", {});

    _defineProperty(this, "next", void 0);

    _defineProperty(this, "timeout", void 0);

    // FIXME: make these private and have getters
    this.state = __parent__.state;
    this.util = new class {
      constructor(event, exec, time) {
        this.event = event;
        this.exec = exec;
        this.time = time;
      }

    }(__parent__.util.event, __parent__.util.exec, __parent__.util.time);
    this.timeout = deadline;
  }
  /**
   * Derived classes must implement this method. Because it is used in a very unusual
   * way -- only by the execution engine, which will apply it to the arguments that
   * were passed into the constructor -- TypeScript will report errors that have to
   * be suppressed by putting //@ts-ignore on the line before the definitions of derived
   * implementations of this method.
   * @param args The arguments to with this function is to be applied.
   */
  //public abstract react(...args:ArgList<T>): void;
  // public late(...args:ArgList<T>): void {
  //     Log.global.warn("Deadline violation occurred!")
  // }
  //    private values: Map<Readable<unknown>, unknown> = new Map();


  doReact() {
    _util.Log.global.debug(">>> Reacting >>> " + this.constructor.name + " >>> " + this.toString()); // Test if this reaction has a deadline which has been violated.
    // This is the case if the reaction has a defined timeout and
    // logical time + timeout < physical time


    _util.Log.global.debug("Timeout: " + this.timeout);

    if (this.timeout && this.util.time.getCurrentTag().getLaterTag(this.timeout).isEarlierThan(new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0))) {
      this.late.apply(this, this.args.tuple);
    } else {
      this.react.apply(this, this.args.tuple); // on time
    }
  }
  /**
   * Setter for reaction deadline. Once a deadline has been set
   * the deadline's timeout will determine whether the reaction's 
   * react function or the deadline's handle function will be invoked.
   * If a deadline has not been set the reaction's react function
   * will be invoked once triggered. 
   * @param deadline The deadline to set to this reaction.
   */


  setDeadline(timeout) {
    this.timeout = timeout;
    return this;
  }
  /**
   * Setter for reaction priority. This should
   * be determined by topological sort of reactions.
   * @param priority The priority for this reaction.
   */


  setPriority(priority) {
    this.priority = priority;
  }

}
/**
 * An event is caused by a timer or a scheduled action. 
 * Each event is tagged with a time instant and may carry a value 
 * of arbitrary type. The tag will determine the event's position
 * with respect to other events in the event queue.
 */


exports.Reaction = Reaction;

class Event {
  /**
   * Constructor for an event.
   * @param trigger The trigger of this event.
   * @param tag The tag at which this event occurs.
   * @param value The value associated with this event. 
   * 
   */
  constructor(trigger, tag, value) {
    this.trigger = trigger;
    this.tag = tag;
    this.value = value;

    _defineProperty(this, "next", void 0);
  }

  hasPriorityOver(node) {
    if (node) {
      return this.getPriority().isEarlierThan(node.getPriority());
    } else {
      return false;
    }
  }

  updateIfDuplicateOf(node) {
    if (node && node instanceof Event) {
      if (this.trigger === node.trigger && this.tag.isSimultaneousWith(node.tag)) {
        node.value = this.value; // update the value

        return true;
      }
    }

    return false;
  } // getID(): [Variable, TimeInstant] {
  //     return [this.trigger, this.time];
  // }


  getNext() {
    return this.next;
  }

  setNext(node) {
    this.next = node;
  }

  getPriority() {
    return this.tag;
  }

}
/**
 * An action denotes a self-scheduled event.
 * An action, like an input, can cause reactions to be invoked.
 * Whereas inputs are provided by other reactors, actions are scheduled
 * by this reactor itself, either in response to some observed external
 * event or as a delayed response to some input event. The action can be
 * scheduled by a reactor by invoking the schedule function in a reaction
 * or in an asynchronous callback that has been set up in a reaction.
 */


class Action extends Descendant {
  //name: string;
  // A value is available to any reaction triggered by this action.
  // The value is not directly associated with a timestamp because
  // every action needs a timestamp (for _isPresent()) and only
  // some actions carry values. 
  // The most recent time this action was scheduled.
  // Used by the isPresent function to tell if this action
  // has been scheduled for the current logical time.
  update(e) {
    if (!e.tag.isSimultaneousWith(this.__parent__.util.time.getCurrentTag())) {
      throw new Error("Time of event does not match current logical time.");
    }

    if (e.trigger == this) {
      //@ts-ignore
      this.value = e.value;
      this.timestamp = e.tag;

      this.__parent__.triggerReactions(e);
    } else {
      throw new Error("Attempt to update action using incompatible event.");
    }
  }
  /**
   * Returns true if this action was scheduled for the current
   * logical time. This result is not affected by whether it
   * has a value.
   */


  isPresent() {
    if (this.timestamp == undefined) {
      // This action has never been scheduled before.
      return false;
    }

    if (this.timestamp.isSimultaneousWith(this.__parent__.util.time.getCurrentTag())) {
      return true;
    } else {
      return false;
    }
  }

  isChildOf(r) {
    if (this.__parent__ && this.__parent__ === r) {
      return true;
    }

    return false;
  }
  /**
   * Called on an action within a reaction to acquire the action's value.
   * The value for an action is set by a scheduled action event, and is only
   * present for reactions executing at that logical time. When logical time
   * advances, that previously available value is now unavailable.
   * If the action was scheduled with no value, this function returns `null`.
   */


  get() {
    if (this.isPresent()) {
      return this.value;
    } else {
      return null;
    }
  }
  /** 
   * Construct a new action.
   * @param __parent__ The reactor containing this action.
   * @param origin Optional. If physical, then the hardware clock on the local 
   * platform is used to determine the tag of the resulting event. If logical, 
   * the current logical time (plus one microstep) is used as the offset.
   * @param minDelay Optional. Defaults to 0. Specifies the intrisic delay of
   * any events resulting from scheduling this action.
   */


  constructor(__parent__, origin, minDelay = new _time.TimeValue(0)) {
    super(__parent__);
    this.__parent__ = __parent__;

    _defineProperty(this, "origin", void 0);

    _defineProperty(this, "minDelay", void 0);

    _defineProperty(this, "value", null);

    _defineProperty(this, "timestamp", void 0);

    this.origin = origin;
    this.minDelay = minDelay;
  }

  toString() {
    return this.getFullyQualifiedName();
  }

  isSchedulable() {
    return false;
  }

}

exports.Action = Action;

class Startup extends Action {
  constructor(__parent__) {
    super(__parent__, _time.Origin.logical);
  }

}

exports.Startup = Startup;

class Shutdown extends Action {
  constructor(__parent__) {
    super(__parent__, _time.Origin.logical);
  }

}

exports.Shutdown = Shutdown;

class Parameter {
  constructor(value) {
    this.value = value;
  }

  get() {
    return this.value;
  }

} // It's valid to create state for a reactor without initializing it to a value,
// so the type of State is T | undefined.


exports.Parameter = Parameter;

class State {
  constructor(value) {
    this.value = value;
  }

  get() {
    return this.value;
  }

  set(value) {
    this.value = value;
  }

}

exports.State = State;

class Scheduler {
  constructor(__parent__, action) {
    this.__parent__ = __parent__;
    this.action = action;
  }

  get() {
    return this.action.get();
  }
  /**
   * Schedule this action. An event for this action will be
   * created and pushed onto the event queue. If the same action
   * is scheduled multiple times for the same logical time, the value
   * associated with the last invocation of the this function determines
   * the value attached to the action at that logical time.
   * @param extraDelay An additional scheduling delay on top of the intrinsic
   * delay of the action. See
   * https://github.com/icyphy/lingua-franca/wiki/Language-Specification#Action-Declaration.
   * @param value An optional value to be attached to this action.
   * The value will be available to reactions depending on this action.
   */


  schedule(extraDelay, value) {
    if (!(extraDelay instanceof _time.TimeValue)) {
      extraDelay = new _time.TimeValue(0);
    }

    var tag = this.__parent__.util.time.getCurrentTag();

    var delay = this.action.minDelay.add(extraDelay);

    if (this.action.origin == _time.Origin.physical) {
      tag = new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0);
    }

    tag = tag.getLaterTag(delay);

    if (this.action.origin == _time.Origin.logical && !(this.action instanceof Startup)) {
      tag = tag.getMicroStepLater();
    }

    this.__parent__.util.event.schedule(new Event(this.action, tag, value)); // For logging


    let actionType;

    if (this.action.origin == _time.Origin.logical) {
      actionType = "logical";
    } else {
      actionType = "physical";
    }

    _util.Log.global.debug("Scheduling " + actionType + " action " + this.action.getName() + " with tag: " + tag);
  }

}
/**
 * A timer is an attribute of a reactor which periodically (or just once)
 * creates a timer event. A timer has an offset and a period. Upon initialization
 * the timer will schedule an event at the given offset from starting wall clock time.
 * Whenever this timer's event comes off the event queue, it will 
 * reschedule the event at the current logical time + period in the future. A 0 
 * period indicates the timer's event is a one-off and should not be rescheduled.
 */


exports.Scheduler = Scheduler;

class Timer extends Descendant {
  get() {
    if (this.tag && this.tag.isSimultaneousWith(this.__parent__.util.time.getCurrentTag())) {
      return this.tag;
    } else {
      return null;
    }
  }

  isPresent() {
    if (this.get() != null) {
      return true;
    }

    return false;
  }

  /**
   * Timer constructor. 
   * @param __parent__ The reactor this timer is attached to.
   * @param offset The interval between the start of execution and the first
   * timer event. Cannot be negative.
   * @param period The interval between rescheduled timer events. If 0, will
   * not reschedule. Cannot be negative.
   */
  constructor(__parent__, offset, period) {
    super(__parent__);
    this.__parent__ = __parent__;

    _defineProperty(this, "tag", void 0);

    _defineProperty(this, "period", void 0);

    _defineProperty(this, "offset", void 0);

    if (!(offset instanceof _time.TimeValue)) {
      this.offset = new _time.TimeValue(0);
    } else {
      this.offset = offset;
    }

    if (!(period instanceof _time.TimeValue)) {
      this.period = new _time.TimeValue(0);
    } else {
      this.period = period;
    }
  }
  /**
   * Update the current value of this timer in accordance with the given
   * event, and trigger any reactions that list this timer as their trigger.
   * @param e Timestamped event.
   */


  update(e) {
    if (!e.tag.isSimultaneousWith(this.__parent__.util.time.getCurrentTag())) {
      throw new Error("Time of event does not match current logical time.");
    }

    if (e.trigger == this) {
      this.tag = e.tag;

      this.__parent__.triggerReactions(e);
    }
  }

  toString() {
    return "Timer from " + this.__parent__.getFullyQualifiedName() + " with period: " + this.period + " offset: " + this.offset;
  }

}

exports.Timer = Timer;

class Mutation extends Reaction {
  /**
   * @override
   */
  toString() {
    return this.__parent__.getFullyQualifiedName() + "[M" + this.__parent__.getMutationIndex(this) + "]";
  }

  constructor(__parent__, trigs, args, react, deadline, late = () => {
    _util.Log.global.warn("Deadline violation occurred!");
  }) {
    // FIXME: make these private and have getters
    super(__parent__, trigs, args, react, deadline, late);
    this.__parent__ = __parent__;
    this.trigs = trigs;
    this.args = args;
    this.react = react;
    this.deadline = deadline;
    this.late = late;

    _defineProperty(this, "util", void 0);

    this.util = __parent__.util;
  }

}

exports.Mutation = Mutation;

class Args {
  constructor(...args) {
    _defineProperty(this, "tuple", void 0);

    this.tuple = args;
  }

}

exports.Args = Args;

class Triggers {
  constructor(trigger, ...triggers) {
    _defineProperty(this, "list", void 0);

    this.list = triggers.concat(trigger);
  }

}
/**
 * A reactor is a software component that reacts to input events,
 * timer events, and action events. It has private state variables
 * that are not visible to any other reactor. Its reactions can
 * consist of altering its own state, sending messages to other
 * reactors, or affecting the environment through some kind of
 * actuation or side effect.
 */


exports.Triggers = Triggers;

class Reactor extends Descendant {
  // FIXME: may create a setter for an alias rather than put a mandatory name in the constructor
  // private _dependsOnPorts: Map<Reaction<unknown>, Set<Port<unknown>>> = new Map();
  // private _dependentOnPorts: Map<Reaction<unknown>, Set<Port<unknown>>> = new Map();
  // FIXME: use these so we can make startup and shutdown private
  isTrigger(trigger) {}
  /**
   * Generic helper function that turns a list of rest parameters into a VarList.
   * It is necessary to pass `args` given to the constructor of a reaction
   * through this function in order to ensure type safety. Otherwise, the 
   * inferred type will collapse to `Array<T>` where `T` is the union type 
   * of all elements found in the list, which is far less specific than the
   * return type inferred for this function.
   * @param args
   * @see {@link https://github.com/Microsoft/TypeScript/pull/24897} 
   * for further information.
   * @see Reaction
   */
  //readonly check = <X extends Variable[]>(...args: X) => args;

  /** Reactions added by the implemented of derived reactor classes. */


  getWritable(port) {
    // FIXME: Implement checks to ensure that port is allowed to be written to.
    return new Writer(port);
  } // protected getTrigger<T extends Readable<ValueOrTime<T,S>>, S extends Present>(variable: T): Readable<ValueOrTime<T,S>> {
  //     return new Trigger(this, variable);
  // }


  getReactionIndex(reaction) {
    for (let i = 0; i < this._reactions.length; i++) {
      if (Object.is(reaction, this._reactions[i])) {
        return i;
      }
    }

    throw new Error("Reaction is not listed.");
  }

  getMutationIndex(mutation) {
    for (let i = 0; i < this._mutations.length; i++) {
      if (Object.is(mutation, this._mutations[i])) {
        return i;
      }
    }

    throw new Error("Mutation is not listed.");
  }
  /**
   * Return the set of downstream ports that this reactor connects 
   * to the given port.
   * @param port The port to look up its destinations for.
   */


  getDestinations(port) {
    if (this.__parent__) {
      let dests = this.__parent__._destinationPorts.get(port);

      if (dests) {
        return dests;
      }
    }

    return new Set();
  }
  /**
   * Return the upstream port that this reactor connects to the given port.
   * @param port The port to look up its source for.
   */


  getSource(port) {
    if (this.__parent__) {
      return this.__parent__._sourcePort.get(port); // FIXME: weird cast
    }
  }
  /**
   * Return the set of reactions within this reactor that are dependent on
   * the given port.
   * @param port The port to look up its depdendent reactions for.
   */


  getDownstreamReactions(port) {
    var reactions = this._dependentReactions.get(port);

    if (reactions) {
      return reactions;
    } else {
      return new Set();
    }
  }
  /**
   * Return the set of reactions within this reactor that the given port 
   * depends on.
   * @param port The port to look up its depdendent reactions for.
   */


  getUpstreamReactions(port) {
    var reactions = this._dependsOnReactions.get(port);

    if (reactions) {
      return reactions;
    } else {
      return new Set();
    }
  }

  getSchedulable(action) {
    return new Scheduler(this, action); /// FIXME: check whether action is local
  } // protected addReaction<T>(reaction: Reaction<T>): void {
  //     // FIXME: We could also construct the reaction in this function.
  //     // That saves having to pass in a reference to `this`.
  //     // Ensure that arguments are compatible with implementation of react().
  //     (function<X>(args: ArgList<X>, fun: (...args:ArgList<X>) => void): void {
  //     })(reaction.args.tuple, reaction.react);
  //     this._reactions.push(reaction);
  //     // Stick this reaction into the trigger map to ensure it gets triggered.
  //     for (let t of reaction.trigs.list) {
  //         let s = this._triggerMap.get(t);
  //         if (s == null) {
  //             s = new Set();
  //             this._triggerMap.set(t, s);
  //         }
  //         s.add(reaction);
  //         // Record this trigger as a dependency.
  //         if (t instanceof Port) {
  //             this._addDependency(t, reaction);
  //         } else {
  //             Log.global.debug(">>>>>>>> not a dependency:" + t); // FIXME: Handle hierarchical references!
  //         }
  //     }
  //     for (let a of reaction.args.tuple) {
  //         if (a instanceof Port) {
  //             if (this._isUpstream(a)) {
  //                 this._addDependency(a, reaction);
  //             } else if (this._isDownstream(a)) {
  //                 this._addAntiDependency(a, reaction);
  //             } else {
  //                 throw new Error("Encountered argument that is neither a dependency nor an antidependency.");
  //             }
  //         }
  //         // Only necessary if we want to add actions to the dependency graph.
  //         if (a instanceof Action) {
  //             // dep
  //         }
  //         if (a instanceof Scheduler) {
  //             // antidep
  //         }
  //         if (a instanceof Writer) {
  //             this._addAntiDependency(a.getPort(), reaction);
  //         }
  //     }
  //     //return reaction;
  // }


  recordDeps(reaction) {
    // Stick this reaction into the trigger map to ensure it gets triggered.
    for (let t of reaction.trigs.list) {
      // If a reaction is triggered by a child reactor's port,
      // it needs to be inserted into the child reactor's trigger map
      // instead of this reactor's trigger map
      let tMap;
      let portParent;

      if (t instanceof Port && !t.isChildOf(this)) {
        // Obtain the child reactor's trigger map
        for (let childReactor of this._getChildren()) {
          if (t.isChildOf(childReactor)) {
            portParent = childReactor;
            break;
          }
        }

        if (portParent === undefined) {
          throw new Error("Port " + t + " is a trigger for reaction " + reaction + " but is neither a child of the reactor containing the reaction" + " or that reactor's children.");
        }

        tMap = portParent._triggerMap;
      } else {
        // Use this reactor's trigger map
        tMap = this._triggerMap;
      }

      let s = tMap.get(t);

      if (s == undefined) {
        s = new Set();
        tMap.set(t, s);
      }

      s.add(reaction); // Record this trigger as a dependency.

      if (t instanceof Port) {
        // The ports of hierarchical references are still given as ports
        this._addDependency(t, reaction);
      } else {
        _util.Log.global.debug(">>>>>>>> not a dependency: " + t);
      }
    }

    for (let a of reaction.args.tuple) {
      if (a instanceof Port) {
        if (this._isUpstream(a)) {
          this._addDependency(a, reaction);
        } else if (this._isDownstream(a)) {
          this._addAntiDependency(a, reaction);
        } else {
          throw new Error("Encountered argument that is neither a dependency nor an antidependency.");
        }
      } // Only necessary if we want to add actions to the dependency graph.


      if (a instanceof Action) {// dep
      }

      if (a instanceof Scheduler) {// antidep
      }

      if (a instanceof Writer) {
        this._addAntiDependency(a.getPort(), reaction);
      }
    }
  }

  addReaction(trigs, args, react, deadline, late = () => {
    _util.Log.global.warn("Deadline violation occurred!");
  }) {
    let reaction = new Reaction(this, trigs, args, react, deadline, late);

    this._reactions.push(reaction);

    this.recordDeps(reaction);
  }

  addMutation(trigs, args, react, deadline, late = () => {
    _util.Log.global.warn("Deadline violation occurred!");
  }) {
    let mutation = new Mutation(this, trigs, args, react, deadline, late);

    this._mutations.push(mutation);

    this.recordDeps(mutation);
  }

  getPrecedenceGraph() {
    var graph = new _util.PrecedenceGraph();

    for (let r of this._getChildren()) {
      graph.merge(r.getPrecedenceGraph());
    }

    let prev = null;

    for (let i = 0; i < this._reactions.length; i++) {
      let r = this._reactions[i];
      graph.addNode(r); // Establish dependencies between reactions
      // depending on their ordering inside the reactor.

      if (prev) {
        graph.addEdge(r, prev);
      }

      var deps = r.getDependencies(); // look upstream

      for (let d of deps[0]) {
        if (d instanceof Port) {
          graph.addEdges(r, d.getUpstreamReactions());
        } else {
          _util.Log.global.error("Found dependency that is not a port");
        }
      } // look downstream


      for (let d of deps[1]) {
        if (d instanceof Port) {
          graph.addBackEdges(r, d.getDownstreamReactions());
        } else {
          _util.Log.global.error("Found antidependency that is not a port");
        }
      }

      prev = r;
    }

    return graph;
  }

  _addDependency(port, reaction) {
    let s = this._dependentReactions.get(port);

    if (s == null) {
      s = new Set();

      this._dependentReactions.set(port, s);
    }

    s.add(reaction);
  }

  _addAntiDependency(port, reaction) {
    let s = this._dependsOnReactions.get(port);

    if (s == null) {
      s = new Set();

      this._dependsOnReactions.set(port, s);
    }

    s.add(reaction);
  }
  /**
   * Assign a value to this port at the current logical time.
   * Put the reactions this port triggers on the reaction 
   * queue and recursively invoke this function on all connected output ports.
   * @param value The value to assign to this port.
   */


  _propagateValue(src) {
    var value = src.get();

    if (value == null) {
      _util.Log.global.debug("Retrieving null value from " + src.getFullyQualifiedName());

      return;
    }

    var reactions = this._triggerMap.get(src); // Push triggered reactions onto the reaction queue.


    if (reactions != undefined) {
      for (let r of reactions) {
        this.app._triggerReaction(r);
      }
    } else {
      _util.Log.global.debug("No reactions to trigger.");
    } // Update all ports that the src is connected to.


    var dests = undefined;

    if (src instanceof InPort) {
      dests = this._destinationPorts.get(src);
    } else if (this.__parent__ && this.__parent__ instanceof Reactor) {
      dests = this.__parent__._destinationPorts.get(src); // FIXME: obtain set of writable object from this map
    }

    if (dests != undefined) {
      for (let d of dests) {
        // The following is type safe because we're doing
        // type checks in connect().
        //@ts-ignore
        d.update(this.getWritable(d), value);
      }
    } else {
      _util.Log.global.debug("No downstream receivers.");
    }
  }

  triggerReactions(e) {
    _util.Log.global.debug("Triggering reactions sensitive to " + e.trigger);

    let reactions = this._triggerMap.get(e.trigger);

    if (reactions) {
      for (let r of reactions) {
        this.app._triggerReaction(r);
      }
    }
  }
  /**
   * Create a new reactor.
   * @param __parent__ Parent of this reactor.
   */


  constructor(__parent__) {
    super(__parent__); // if (alias) {
    //     this.setAlias(alias);
    // }

    _defineProperty(this, "_isActive", false);

    _defineProperty(this, "state", {});

    _defineProperty(this, "_triggerMap", new Map());

    _defineProperty(this, "_dependsOnReactions", new Map());

    _defineProperty(this, "_dependentReactions", new Map());

    _defineProperty(this, "_sourcePort", new Map());

    _defineProperty(this, "_destinationPorts", new Map());

    _defineProperty(this, "_startupActions", new Set());

    _defineProperty(this, "_shutdownActions", new Set());

    _defineProperty(this, "_reactions", []);

    _defineProperty(this, "_mutations", []);

    _defineProperty(this, "startup", new Startup(this));

    _defineProperty(this, "shutdown", new Shutdown(this));

    _defineProperty(this, "app", void 0);

    _defineProperty(this, "util", void 0);

    _defineProperty(this, "_priority", -1);

    if (__parent__ != null) {
      this.app = __parent__.app;
    } else {
      if (this instanceof App) {
        this.app = this;
      } else {
        throw new Error("Cannot instantate reactor without a parent.");
      }
    } // Even though TypeScript doesn't catch it, the following statement
    // will assign `undefined` if the this is an instance of App.


    this.util = this.app.util; // NOTE: beware, if this is an instance of App, `this.util` will be `undefined`.
    // Do not attempt to reference during the construction of an App.

    if (!(this instanceof App)) {
      // Add default startup reaction.
      this.addMutation(new Triggers(this.startup), new Args(), function () {
        var reactor = this.__parent__; // FIXME: make parent private and add
        // function below part of ReactorUtils
        //Log.global.log("*** Starting up reactor " + reactor.getFullyQualifiedName());
        // Schedule startup for all contained reactors.

        reactor._startupChildren();

        reactor._setTimers();

        reactor._isActive = true;
      }); // Add default shutdown reaction.

      this.addMutation(new Triggers(this.shutdown), new Args(), function () {
        var reactor = this.__parent__;

        _util.Log.global.log("*** Shutting down reactor " + reactor.getFullyQualifiedName());

        reactor._unsetTimers(); // Schedule shutdown for all contained reactors.


        reactor._shutdownChildren();

        reactor._isActive = false;
      });
    }
  }

  _startupChildren() {
    for (let r of this._getChildren()) {
      _util.Log.global.debug("Propagating startup: " + r.startup); // Note that startup reactions are scheduled without a microstep delay


      this.getSchedulable(r.startup).schedule(0);
    }
  }

  _shutdownChildren() {
    _util.Log.global.debug("Shutdown children was called");

    for (let r of this._getChildren()) {
      _util.Log.global.debug("Propagating shutdown: " + r.shutdown);

      this.getSchedulable(r.shutdown).schedule(0);
    }
  }

  isChildOf(parent) {
    if (this.__parent__ === parent) {
      return true;
    } else {
      return false;
    }
  }
  /**
   * Obtain the set of this reactor's child reactors.
   * Watch out for cycles!
   * This function ignores reactor attributes which reference
   * this reactor's parent and this reactor's app.
   * It is an error for a reactor to be a child of itself.
   */


  _getChildren() {
    let children = new Set();

    for (const [key, value] of Object.entries(this)) {
      // If pointers to other non-child reactors in the hierarchy are not
      // excluded (eg. value != this.parent) this function will loop forever.
      if (value instanceof Reactor && value != this.__parent__ && !(value instanceof App)) {
        // A reactor may not be a child of itself.
        if (value == this) {
          throw new Error("A reactor may not have itself as an attribute." + " Reactor attributes of a reactor represent children" + " and a reactor may not be a child of itself");
        }

        children.add(value);
      }
    }

    return children;
  } // public _registerStartupShutdown(startup: Startup, shutdown: Action<unknown>) {
  //     // FIXME: do hierarchy check to ensure that this reactors should have access to these actions.
  //     this._startupActions.add(startup);
  //     this._shutdownActions.add(shutdown);
  // }

  /**
   * Returns the set of reactions owned by this reactor.
   */


  _getReactions() {
    var set = new Set();

    for (let entry of this._reactions) {
      set.add(entry);
    }

    return set;
  }

  _isDownstream(arg) {
    if (arg instanceof InPort) {
      if (arg.isGrandChildOf(this)) {
        return true;
      }
    } else {
      if (arg.isChildOf(this)) {
        return true;
      }
    }

    return false;
  }

  _isUpstream(arg) {
    if (arg instanceof OutPort) {
      if (arg.isGrandChildOf(this)) {
        return true;
      }
    } else {
      if (arg.isChildOf(this)) {
        return true;
      }
    }

    return false;
  } // public _inScope(var: Variable<any>): boolean {
  //     return false;
  // }

  /**
   * Returns true if a given source port can be connected to the
   * given destination port. False otherwise. Valid connections
   * must:
   * (1) satisfy particular hierachical constraints following 
   * from the scope rules of reactors; and
   * (2) not introduce cycles.
   * @param src The start point of the tried connection.
   * @param dst The end point of the tried connection.
   */


  canConnect(src, dst) {
    // 1. Rule out self loops. 
    //   - (including trivial ones)
    if (src === dst) {
      return false;
    } // FIXME: check the local dependency graph to figure out whether this
    // change introduces zero-delay feedback.
    // 2. Rule out write conflicts.
    //   - (between reactors)


    if (this._sourcePort.get(dst) != undefined) {
      return false;
    } //   - between reactors and reactions 
    // (NOTE: check also needs to happen in addReaction)


    var antideps = this._dependsOnReactions.get(dst);

    if (antideps != undefined && antideps.size > 0) {
      return false;
    } // 3. Assure that the scoping rules are adhered to.


    if (src instanceof OutPort) {
      if (dst instanceof InPort) {
        // OUT to IN
        if (src.isGrandChildOf(this) && dst.isGrandChildOf(this)) {
          return true;
        } else {
          return false;
        }
      } else {
        // OUT to OUT
        if (src.isGrandChildOf(this) && dst.isChildOf(this)) {
          return true;
        } else {
          return false;
        }
      }
    } else {
      if (src === dst) {
        return false;
      }

      if (dst instanceof InPort) {
        // IN to IN
        if (src.isChildOf(this) && dst.isGrandChildOf(this)) {
          return true;
        } else {
          return false;
        }
      } else {
        // IN to OUT
        return false;
      }
    }
  }

  _connect(src, dst) {
    //***********
    if (this.canConnect(src, dst)) {
      _util.Log.global.debug("connecting " + src + " and " + dst);

      let dests = this._destinationPorts.get(src);

      if (dests == null) {
        dests = new Set();
      }

      dests.add(dst);

      this._destinationPorts.set(src, dests);

      this._sourcePort.set(dst, src);
    } else {
      throw new Error("ERROR connecting " + src + " to " + dst);
    }
  }

  _disconnect(src, dst) {
    _util.Log.global.debug("disconnecting " + src + " and " + dst);

    let dests = this._destinationPorts.get(src);

    if (dests != null) {
      dests.delete(dst);
    }

    this._sourcePort.delete(src);
  }
  /**
   * Returns the set of reactions directly owned by this reactor combined with 
   * the recursive set of all reactions of contained reactors.
   */
  // public _getAllReactions(): Set<Reaction<unknown>> {
  //     let reactions = this._getReactions();
  //     // Recursively call this function on child reactors
  //     // and add their timers to the timers set.
  //     let children = this._getChildren();
  //     if(children){
  //         for(const child of children){
  //             if(child){
  //                 let subReactions = child._getAllReactions();
  //                 for(const subReaction of subReactions){
  //                     reactions.add(subReaction);
  //                 }                     
  //             }
  //         }
  //     }
  //     return reactions;
  // }

  /**
   * Set all the timers of this reactor.
   */


  _setTimers() {
    _util.Log.global.debug("Setting timers for: " + this);

    let timers = new Set();

    for (const [k, v] of Object.entries(this)) {
      if (v instanceof Timer) {
        this.app._setTimer(v);
      }
    }
  }
  /**
   * Unset all the timers of this reactor.
   */


  _unsetTimers() {
    // Log.global.debug("Getting timers for: " + this)
    let timers = new Set();

    for (const [k, v] of Object.entries(this)) {
      if (v instanceof Timer) {
        this.app._unsetTimer(v);
      }
    }
  }
  /**
   * Iterate through this reactor's attributes,
   * and return the set of its ports.
   */


  _getPorts() {
    // Log.global.debug("Getting ports for: " + this)
    let ports = new Set();

    for (const [key, value] of Object.entries(this)) {
      if (value instanceof Port) {
        ports.add(value);
      }
    }

    return ports;
  }
  /**
   * Iterate through this reactor's attributes,
   * and return the set of its actions.
   */


  _getActions() {
    // Log.global.debug("Getting actions for: " + this)
    let actions = new Set();

    for (const [key, value] of Object.entries(this)) {
      if (value instanceof Action) {
        actions.add(value);
      }
    }

    return actions;
  } //A reactor's priority represents its order in the topological sort.
  //The default value of -1 indicates a priority has not been set.


  //FIXME: assign in constructor?
  toString() {
    return this.getFullyQualifiedName();
  }
  /**
   * Recursively sets the app attribute for this reactor and all contained reactors to app.
   * @param app The app for this and all contained reactors.
   */
  // public _setApp(app: App){
  //     // Log.global.debug("Starting _setApp for: " + this._getFullyQualifiedName());
  //     Log.global.debug("Setting app for: " + this);
  //     this._app = app;
  //     // Recursively set the app attribute for all contained reactors to app.
  //     let children = this._getChildren();
  //     if(children){
  //         for(let child of children){
  //             child._setApp(app);
  //         }
  //     }
  // }

  /**
   * Recursively traverse all reactors and verify the 
   * parent property of each component correctly matches its location in
   * the reactor hierarchy.
   */


  _checkAllParents(parent) {
    if (this.__parent__ != parent) throw new Error("The parent property for " + this + " does not match the reactor hierarchy."); // FIXME: check that there exist no copies?
    // This might be difficult...

    let children = this._getChildren();

    for (let child of children) {
      child._checkAllParents(this);
    } // Ports have their parent set in constructor, so verify this was done correctly.


    let ports = this._getPorts();

    for (let port of ports) {
      if (!port.isChildOf(this)) {
        throw new Error("A port has been incorrectly constructed as an attribute of " + "a different reactor than the parent it was given in its constructor: " + port);
      }
    }

    let actions = this._getActions();

    for (let action of actions) {
      if (!action.isChildOf(this)) throw new Error("The parent property for " + action + " does not match the reactor hierarchy.");
    }
  }

}

exports.Reactor = Reactor;

class Port extends Descendant {
  /** The time stamp associated with this port's value. */

  /** The value associated with this port. */
  // public _connectedSinkPorts: Set<Port<unknown>> = new Set<Port<T>>(); // FIXME: change this into a private map hosted in the reactor
  // public _connectedSourcePort: Port<T>| null = null; // FIXME: change this into a private map hosted in the reactor

  /**
   * Return the transitive closure of reactions dependent on this port.
   */
  getDownstreamReactions() {
    var reactions = new Set();

    for (let d of this.__parent__.getDestinations(this)) {
      reactions = new Set([...reactions, ...d.getDownstreamReactions()]);
    }

    reactions = new Set([...reactions, ...this.__parent__.getDownstreamReactions(this)]);
    return reactions;
  }
  /**
   * Return the transitive closure of reactions dependent on this port.
   */


  getUpstreamReactions() {
    var reactions = new Set();

    var source = this.__parent__.getSource(this);

    _util.Log.global.debug("Finding upstream reactions for " + this);

    if (source) {
      _util.Log.global.debug(">>>"); // Reactions upstream (i.e., in other reactors).


      reactions = new Set([...reactions, ...source.getUpstreamReactions()]);
    } // Reactions local (i.e., within the reactor).


    reactions = new Set([...reactions, ...this.__parent__.getUpstreamReactions(this)]);

    _util.Log.global.debug("Reactions found!");

    return reactions;
  }

  isChildOf(r) {
    if (this.__parent__ && this.__parent__ === r) {
      return true;
    }

    return false;
  }

  isGrandChildOf(r) {
    if (this.__parent__ && this.__parent__.isChildOf(r)) {
      return true;
    }

    return false;
  }
  /**
   * Returns true if the connected port's value has been set.
   * an output port or an input port with a value set at the current logical time
   * Returns false otherwise
   */


  isPresent() {
    _util.Log.global.debug("In isPresent()...");

    _util.Log.global.debug("value: " + this.value);

    _util.Log.global.debug("tag: " + this.tag);

    _util.Log.global.debug("time: " + this.__parent__.util.time.getCurrentLogicalTime());

    if (this.value != null && this.tag != undefined && this.tag.isSimultaneousWith(this.__parent__.util.time.getCurrentTag())) {
      return true;
    } else {
      return false;
    }
  }

  update(writer, value) {
    if (writer.isProxyOf(this)) {
      // Only update the value if the proxy has a reference
      // to this port. If it does, the type variables must
      // match; no further checks are needed.
      _util.Log.global.debug("Updating value of " + this.getFullyQualifiedName()); //@ts-ignore


      this.value = value;

      _util.Log.global.debug(">> parent: " + this.__parent__);

      this.tag = this.__parent__.util.time.getCurrentTag();

      this.__parent__._propagateValue(this); // FIXME: should this be a utility function?

    } else {
      _util.Log.global.warn("WARNING: port update denied.");
    }
  }
  /**
   * Obtains the value set to this port. Values are either set directly by calling set()
   * on this port, or indirectly by calling set() on a connected upstream port.
   * Will return null if the connected output did not have its value set at the current
   * logical time.
   */


  get() {
    if (this.isPresent()) {
      return this.value;
    } else {
      return null;
    }
  }
  /**
   * Create a new port on the given reactor.
   * @param __parent__ 
   */


  constructor(__parent__) {
    super(__parent__);
    this.__parent__ = __parent__;

    _defineProperty(this, "tag", void 0);

    _defineProperty(this, "value", null);
  }

  toString() {
    return this.getFullyQualifiedName();
  }

}

exports.Port = Port;

class OutPort extends Port {
  toString() {
    return this.getFullyQualifiedName();
  }

}

exports.OutPort = OutPort;

class InPort extends Port {
  toString() {
    return this.getFullyQualifiedName();
  }

}

exports.InPort = InPort;

class Writer {
  // NOTE: don't export this class!
  constructor(port) {
    this.port = port;
  }
  /**
  * Write a value and recursively transmit it to connected ports, which may
  * trigger downstream reactions. No action is taken if the given value is
  * null.
  * @param value The value to be written.
  */


  set(value) {
    _util.Log.global.debug("set() has been called on " + this.port.getFullyQualifiedName());

    if (value != null) {
      this.port.update(this, value);
    }
  }

  get() {
    return this.port.get();
  }

  isProxyOf(port) {
    if (this.port === port) {
      return true;
    }

    return false;
  }

  getPort() {
    return this.port;
  }

  toString() {
    return "Writable(" + this.port.toString() + ")";
  }

}

class EventQueue extends _util.PrioritySet {
  push(event) {
    return super.push(event);
  }

  pop() {
    return super.pop();
  }

  peek() {
    return super.peek();
  }

}

class ReactionQueue extends _util.PrioritySet {
  push(reaction) {
    return super.push(reaction);
  }

  pop() {
    return super.pop();
  }

  peek() {
    return super.peek();
  }

}

class App extends Reactor {
  // Perhaps make this an abstract class, like reactor; omit the name parameter.

  /**
   * The current time, made available so actions may be scheduled relative to it.
   */

  /**
   * The identifier for the timeout used to ensure alarms execute asynchronously.
   */

  /**
   * The time when the alarm's timeout will occur.
   */

  /**
   * Priority set that keeps track of scheduled events.
   */

  /**
   * If not null, finish execution with success, this time interval after
   * the start of execution.
   */

  /**
   * The time at which normal execution should terminate. Note: execution continues for one 
   * more microstep, to give shutdown reactions a chance to execute.
   * Is initially set to the app's timeout + _startOfExecution or undefined if no timeout
   * is given. When _shutdown is called, _endOfExecution is updated to the current time.
   */

  /**
   * If false, execute with normal delays to allow physical time to catch up to logical time.
   * If true, don't wait for physical time to match logical time.
   */

  /**
   * Indicates whether the program should continue running once the event 
   * queue is empty.
   */

  /**
   * Priority set that keeps track of reactions at the current Logical time.
   */

  /**
   * Indicates whether the app is shutting down.
   * It is important not to schedule multiple/infinite shutdown events for the app.
   */

  /**
   * The physical time when execution began relative to January 1, 1970 00:00:00 UTC.
   * Initialized in start().
   */

  /**
   * Report a timer to the app so that it gets scheduled.
   * @param timer The timer to report to the app.
   */
  _setTimer(timer) {
    _util.Log.global.debug(">>>>>>>>>>>>>>>>>>>>>>>>Setting timer: " + timer);

    let startTime;

    if (timer.offset.isZero()) {
      // getLaterTime always returns a microstep of zero, so handle the
      // zero offset case explicitly.
      startTime = this.util.time.getCurrentTag().getMicroStepLater();
    } else {
      startTime = this.util.time.getCurrentTag().getLaterTag(timer.offset);
    }

    this.schedule(new Event(timer, startTime, null));
  }
  /**
   * Report a timer to the app so that it gets unscheduled.
   * @param timer The timer to report to the app.
   */


  _unsetTimer(timer) {} // push a new event onto the event queue
  // FIXME: we could either set the timer to 'inactive' to tell the 
  // scheduler to ignore future event and prevent it from rescheduling any.
  // The problem with this approach is that if, for some reason, a timer would get
  // reactivated, it could start seeing events that were scheduled prior to its
  // becoming inactive. Alternatively, we could remove the event from the queue, 
  // but we'd have to add functionality for this.
  //private heartbeat: Timer = new Timer(this, 0, new TimeInterval(5));


  /**
   * Create a new top-level reactor.
   * @param executionTimeout Optional parameter to let the execution of the app time out.
   * @param keepAlive Optional parameter, if true allows execution to continue with an empty event queue.
   * @param fast Optional parameter, if true does not wait for physical time to catch up to logical time.
   * @param success Optional callback to be used to indicate a successful execution.
   * @param failure Optional callback to be used to indicate a failed execution.
   */
  constructor(executionTimeout = undefined, keepAlive = false, fast = false, success = () => {}, failure = () => {
    throw new Error("Default app failure callback");
  }) {
    super(null); //App.instances.add(this);

    this.success = success;
    this.failure = failure;

    _defineProperty(this, "util", new class {
      constructor(event, exec, graph, time) {
        this.event = event;
        this.exec = exec;
        this.graph = graph;
        this.time = time;
      }

    }(new class {
      constructor(app) {
        this.app = app;
      }

      schedule(e) {
        return this.app.schedule(e);
      }

    }(this), new class {
      constructor(app) {
        this.app = app;
      }

      requestShutdown() {
        this.app._shutdown();
      }

      success() {
        return this.app.success();
      }

      failure() {
        return this.app.failure();
      }

    }(this), new class {
      constructor(app) {
        this.app = app;
      }

      connect(src, dst) {
        return this.app._connect(src, dst);
      }

    }(this), new class {
      constructor(app) {
        this.app = app;
      }

      getCurrentTag() {
        return this.app._currentTag;
      }

      getCurrentLogicalTime() {
        return this.app._currentTag.time;
      }

      getCurrentPhysicalTime() {
        return (0, _time.getCurrentPhysicalTime)();
      }

      getElapsedLogicalTime() {
        return this.app._currentTag.getTimeDifference(this.app._startOfExecution);
      }

      getElapsedPhysicalTime() {
        return (0, _time.getCurrentPhysicalTime)().subtract(this.app._startOfExecution.time);
      }

    }(this)));

    _defineProperty(this, "_currentTag", void 0);

    _defineProperty(this, "alarm", new NanoTimer());

    _defineProperty(this, "_alarmTimeoutID", void 0);

    _defineProperty(this, "_currentAlarmTime", void 0);

    _defineProperty(this, "_eventQ", new EventQueue());

    _defineProperty(this, "_executionTimeout", void 0);

    _defineProperty(this, "_endOfExecution", void 0);

    _defineProperty(this, "_fast", void 0);

    _defineProperty(this, "_keepAlive", false);

    _defineProperty(this, "_reactionQ", new ReactionQueue());

    _defineProperty(this, "_shutdownStarted", false);

    _defineProperty(this, "_startOfExecution", void 0);

    _defineProperty(this, "snooze", new Action(this, _time.Origin.logical, new _time.TimeValue(1, 0)));

    this._executionTimeout = executionTimeout;
    this._keepAlive = keepAlive;
    this._fast = fast; // NOTE: these will be reset properly during startup.

    this._currentTag = new _time.Tag(new _time.TimeValue(0), 0);
    this._startOfExecution = this._currentTag; // Add default startup reaction.

    this.addMutation(new Triggers(this.startup), new Args(), function () {
      //Log.global.log("*** Starting up reactor " + (this.__parent__ as Reactor).getFullyQualifiedName());
      // If the end of execution is known at startup, schedule a 
      // shutdown event to that effect.
      // Note that we schedule shutdown one microstep later, so that
      // any event scheduled exactly at the end of execution will be
      // handled before the shutdown sequence starts.
      var app = this.__parent__;
      var eoe = app._endOfExecution;

      if (eoe) {
        app.schedule(new Event(app.shutdown, eoe.getMicroStepLater(), null));
      } // Schedule startup for all contained reactors.


      app._startupChildren();

      app._setTimers();

      app._isActive = true;
    }); // Add default shutdown reaction.

    this.addMutation(new Triggers(this.shutdown), new Args(), function () {
      var app = this.__parent__; //Log.global.log("*** Shutting down reactor " + app.getFullyQualifiedName());

      app._unsetTimers(); // Schedule shutdown for all contained reactors.


      app._shutdownChildren();

      app._isActive = false;
    });
  }

  // FIXME: we have to remove the instance from the set when we're done with it, or this will create a memory leak.
  // getName(): string {
  //     var alias = super.getName();
  //     var count = 0;
  //     var suffix = "";
  //     if (alias == this.constructor.name) {
  //         for (let a of App.instances) {
  //             if (a !== this && alias === a.constructor.name) {
  //                 count++;
  //             }
  //         }
  //     }
  //     if (count > 0) {
  //         suffix = "(" + count + ")";
  //     }
  //     return alias + suffix;
  // }

  /**
   * Handle the next event on the event queue.
   * ----
   * Wait until physical time matches or exceeds the time of the least tag
   * on the event queue. After this wait, advance current_time to match
   * this tag. Then pop the next event(s) from the event queue that all 
   * have the same tag, and extract from those events the reactions that
   * are to be invoked at this logical time.
   * Sort those reactions by index (determined by a topological sort)
   * and then execute the reactions in order. Each reaction may produce
   * outputs, which places additional reactions into the index-ordered
   * priority queue. All of those will also be executed in order of indices.
   * If the execution timeout given to this app in its constructor
   * has a non-null value, then call successCallback (and end this loop) 
   * when the logical time from the start of execution matches the
   * specified duration. If execution timeout is null, execution will be
   * allowed to continue indefinitely.
   */
  _next(event) {
    _util.Log.global.debug("New invocation of next()."); // Log.global.info(">>> Logical time elapsed: " + this.util.time.getElapsedLogicalTime().getNanoTime());
    // Log.global.info(">>> Physical time elapsed: " + this.util.time.getElapsedPhysicalTime().getNanoTime());


    _util.Log.global.debug("{{{ Triggered by event: " + event.trigger + " }}}");

    if (this._endOfExecution && this._endOfExecution.isEarlierThan(this._currentTag) && !this._endOfExecution.getTimeDifference(event.tag).isZero()) {
      // Remove remaining items from the event queue, if any.
      this._eventQ.empty(); // Clear timeouts, if any.


      this.clearAlarm();

      this._terminateWithSuccess();

      return;
    }

    if (event.trigger === this.snooze) {
      _util.Log.global.debug("Woken up after suspend.");
    }

    let physicalTimeTag = new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0);

    if (physicalTimeTag.isEarlierThan(event.tag) && !this._fast) {
      _util.Log.global.debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Too early to handle next event.");

      _util.Log.global.debug("Time difference: " + physicalTimeTag.getTimeDifference(event.tag).getNanoTime());

      this.setAlarm(event);
      return;
    } // Store the previous tag.


    let previousTag = this._currentTag; // Advance logical time.

    this._currentTag = event.tag;

    if (previousTag.isEarlierThan(event.tag)) {
      _util.Log.global.log(_util.Log.hr);

      _util.Log.global.log(">>> Next event time: " + event.tag);

      _util.Log.global.log(">>> Logical time elapsed: " + this.util.time.getElapsedLogicalTime().getNanoTime());

      _util.Log.global.log(">>> Physical time elapsed: " + this.util.time.getElapsedPhysicalTime().getNanoTime());

      _util.Log.global.log(_util.Log.hr);
    }

    _util.Log.global.debug("Processing events.");

    var nextEvent = event;

    while (nextEvent != null && nextEvent.tag.isSimultaneousWith(this._currentTag)) {
      this._eventQ.pop();

      _util.Log.global.debug("Popped off the event queue: " + nextEvent.trigger); // Handle timers.


      if (nextEvent.trigger instanceof Timer) {
        if (!nextEvent.trigger.period.isZero()) {
          _util.Log.global.debug("Rescheduling timer " + nextEvent.trigger); // reschedule


          this.schedule(new Event(nextEvent.trigger, this._currentTag.getLaterTag(nextEvent.trigger.period), null));
        }
      } // Load reactions onto the reaction queue.


      nextEvent.trigger.update(nextEvent); // Look at the next event on the queue.

      nextEvent = this._eventQ.peek();
    }

    while (this._reactionQ.size() > 0) {
      // FIXME: relevant for mutations:
      // Check whether the reactor is active or not
      // If it is inactive, all reactions, except for those
      // in response to startup actions, should be ignored.
      this._reactionQ.pop().doReact();
    }

    _util.Log.global.debug("Finished handling all events at current time."); // Peek at the event queue again since it may have changed.


    nextEvent = this._eventQ.peek();

    if (nextEvent) {
      _util.Log.global.debug("Event queue not empty.");

      if (nextEvent.trigger !== this.shutdown || this._keepAlive || this._currentTag.time.isEqualTo(event.tag.time)) {
        // Set an alarm to handle the next event.
        _util.Log.global.debug("Setting alarm to wake up for next event.");

        _util.Log.global.log(">>> Logical time elapsed: " + this.util.time.getElapsedLogicalTime().getNanoTime());

        _util.Log.global.log(">>> Physical time elapsed: " + this.util.time.getElapsedPhysicalTime().getNanoTime());

        this.setAlarm(nextEvent);
      } else {
        // The next event is shutdown, 
        // and is scheduled in the future, 
        // but there is nothing else to do.
        _util.Log.global.info("Preponing shutdown.");

        this._eventQ.pop();

        this._shutdown();
      }
    } else {
      // Or suspend/terminate.
      _util.Log.global.debug("Empty event queue.");

      this.suspendOrTerminate();
    }
  }

  suspendOrTerminate() {
    // We have an empty event queue.
    if (this._endOfExecution) {
      // An end of execution was specified;
      // all shutdown events must have been consumed.
      this.clearAlarm();

      this._terminateWithSuccess();
    } else {
      // No end of execution has been specified.
      if (this._keepAlive) {
        // Keep alive: snooze and wake up later.
        _util.Log.global.debug("Going to sleep.");

        this.getSchedulable(this.snooze).schedule(0, this._currentTag);
      } else {
        // Don't keep alive: initiate shutdown.
        _util.Log.global.debug("Initiating shutdown.");

        this._shutdown();
      }
    }
  }
  /**
   * Public method to push events on the event queue. 
   * @param e Prioritized event to push onto the event queue.
   */


  schedule(e) {
    let head = this._eventQ.peek();

    this._eventQ.push(e);

    _util.Log.global.debug("Scheduling with trigger: " + e.trigger);

    _util.Log.global.debug("Elapsed logical time in schedule: " + this.util.time.getElapsedLogicalTime());

    _util.Log.global.debug("Elapsed physical time in schedule: " + this.util.time.getElapsedPhysicalTime());

    if (head == undefined || e.tag.isEarlierThan(head.tag)) {
      this.setAlarm(e);
    }
  }

  clearAlarm() {
    this._currentAlarmTime = undefined;

    if (this._alarmTimeoutID) {
      clearTimeout(this._alarmTimeoutID);
      this._alarmTimeoutID = undefined;
    }

    this.alarm.clearTimeout();
  }

  setAlarm(e) {
    // Note, end of execution is currently set by the app timeout or _shutdown
    if (this._endOfExecution && this._endOfExecution.getMicroStepLater().isEarlierThan(e.tag)) {
      _util.Log.global.debug("Ignoring call to set alarm for event with trigger: " + e.trigger + " because the event's time: " + e.tag + " is after the end of execution (plus a microstep): " + this._endOfExecution.getMicroStepLater());

      this._terminateWithSuccess();
    } else {
      this.clearAlarm();

      _util.Log.global.debug("Setting alarm for event with trigger: " + e.trigger);

      this._currentAlarmTime = e.tag;
      let physicalTimeTag = new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0);

      if (physicalTimeTag.isEarlierThan(e.tag) && !this._fast) {
        let timeout = e.tag.getTimeDifference(physicalTimeTag); // The code below looks funny, but there's an important explanation.
        // Inspecting nanotimer's source at 
        // https://github.com/Krb686/nanotimer/blob/master/lib/nanotimer.js 
        // reveals it internally uses window.setTimeout() for delays over 25ms
        // and synchronously invokes the task for delays less than that.
        // Synchronously calling next is very bad here because it can produce
        // bugs involving the reaction queue not being emptied before starting 
        // _next again resulting in reaction execution at the wrong logical time.
        // It can also lead to a stack overflow. Wrapping alarm.setTimeout
        // in a window.setTimeout with a timeout of 0 forces nanotimer to always
        // act asynchronously.

        this._alarmTimeoutID = setTimeout(function () {
          let physicalTimeTag = new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0);
          let timeout = e.tag.getTimeDifference(physicalTimeTag);
          this.alarm.setTimeout(this._next.bind(this), [e], timeout.getNanoTime());
        }.bind(this), 0);

        _util.Log.global.debug("Timeout: " + timeout.getNanoTime());
      } else {
        this._alarmTimeoutID = setTimeout(function () {
          this._next(e);
        }.bind(this), 0);
      }
    }
  }
  /**
   * Public method to push reaction on the reaction queue. 
   * @param e Prioritized reaction to push onto the reaction queue.
   */


  _triggerReaction(r) {
    _util.Log.global.debug("Pushing " + r + " onto the reaction queue.");

    this._reactionQ.push(r);
  }
  /**
   * Schedule a shutdown event for the app if a shutdown event.
   * has not already been scheduled. Clear the alarm, and set
   * the end of execution to one microstep in the future. 
   */


  _shutdown() {
    if (!this._shutdownStarted) {
      this._shutdownStarted = true;

      _util.Log.global.debug("Initiating shutdown sequence.");

      this._endOfExecution = this._currentTag;

      _util.Log.global.debug("Setting end of execution to: " + this._endOfExecution);

      if (this._currentAlarmTime && this._endOfExecution.getMicroStepLater().isEarlierThan(this._currentAlarmTime)) {
        this.clearAlarm();
      }

      this.getSchedulable(this.shutdown).schedule(0);
    } else {
      _util.Log.global.debug("Ignoring App._shutdown() call after shutdown has already started.");
    }
  }

  _terminateWithSuccess() {
    _util.Log.global.info(_util.Log.hr);

    _util.Log.global.info(">>> End of execution at (logical) time: " + this.util.time.getCurrentLogicalTime());

    _util.Log.global.info(">>> Elapsed physical time: " + this.util.time.getElapsedPhysicalTime());

    _util.Log.global.info(_util.Log.hr);

    this.success();
  }

  _start() {
    _util.Log.global.debug("Initiating startup sequence."); // Recursively check the parent attribute for this and all contained reactors and
    // and components, i.e. ports, actions, and timers have been set correctly.


    this._checkAllParents(null); // Obtain the precedence graph, ensure it has no cycles, 
    // and assign a priority to each reaction in the graph.


    var apg = this.getPrecedenceGraph();

    if (apg.updatePriorities()) {
      _util.Log.global.debug("No cycles.");
    } else {
      throw new Error("Cycle in reaction graph.");
    }

    _util.Log.global.debug(apg.toString()); // Let the start of the execution be the current physical time.


    this._startOfExecution = new _time.Tag((0, _time.getCurrentPhysicalTime)(), 0);
    this._currentTag = this._startOfExecution;

    if (this._executionTimeout != null) {
      this._endOfExecution = this._startOfExecution.getLaterTag(this._executionTimeout);

      _util.Log.global.debug("Execution timeout: " + this._executionTimeout);
    }

    _util.Log.global.info(_util.Log.hr);

    _util.Log.global.info(">>> Start of execution: " + this._currentTag);

    _util.Log.global.info(_util.Log.hr); // Set in motion the execution of this program by scheduling startup at the current logical time.


    this.getSchedulable(this.startup).schedule(0);
  }

}

exports.App = App;

_defineProperty(App, "instances", new Set());