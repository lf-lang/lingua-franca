/**
 * Core of the reactor runtime.
 * 
 * @author Marten Lohstroh (marten@berkeley.edu),
 * @author Matt Weber (matt.weber@berkeley.edu)
 */

import {PrecedenceGraphNode, PrioritySetNode, PrioritySet, PrecedenceGraph} from './util';
import {TimeInterval, TimeInstant, Origin, getCurrentPhysicalTime} from './time';

//---------------------------------------------------------------------//
// Modules                                                             //
//---------------------------------------------------------------------//

/**
 * Timer used for precisely timing the triggering of reactions.
 */
const NanoTimer = require('nanotimer');

/**
 * Logging facilty that has multiple levels of severity.
 * Set the `LOG` environment variable to the desired log
 * level prior to running node. For example:
 * `LOG=info && node ./myapp.js`
 */
var log = require('ulog')('reactor');

//---------------------------------------------------------------------//
// Types and Helper Functions                                          //
//---------------------------------------------------------------------//

/**
 * A variable is a port, action, or timer. Its value is readable using
 * `get`, and may be writable using `set`. When `isPresent` is called 
 * on a variable, it will return true if the value has been set at the 
 * current logical time, and false otherwise.
 */
export type Variable = Readable<unknown> | Writable<unknown> |  { [name: string]: (Readable<unknown> | Writable<unknown>)};

/**
 * A number that indicates a reaction's position with respect to other
 * reactions in an acyclic precendence graph.
 * @see ReactionQueue
 */
export type Priority = number;

/**
 * Conditional type for lists of variables. If the type variable `T` is 
 * inferred to be a subtype of `Variable[]`, it will yield that type, 
 * `never` otherwise.
 */
export type VarList<T> = T extends Variable[] ? T : never;

//---------------------------------------------------------------------//
// Runtime Functions                                                   //
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
// Interfaces                                                          //
//---------------------------------------------------------------------//

/**
 * Collection of functions available to mutations (not reactions).
 */
export interface Mutations {
    connect<D, S extends D>(src:Port<S>, dst:Port<D>): void;
}

export interface Util {
    getCurrentLogicalTime(): TimeInstant;

    getCurrentPhysicalTime(): TimeInstant;

    getElapsedLogicalTime(): TimeInterval;

    getElapsedPhysicalTime(): TimeInterval;

    success(): void; // Not convinced that these should go here

    failure(): void; // ...

    schedule(e: Event<any>): void;
}


/**
 * Interface for readable ports or actions.
 */
export interface Readable<T> {
    get: () => T | null;
    isPresent: () => boolean;
}

/**
 * Interface for schedulable actions.
 */
export interface Schedulable<T> {
    schedule: (extraDelay: TimeInterval | 0, value?: T) => void;
}

/**
 * Interface for writable ports.
 */
export interface Writable<T> extends Readable<T> {
    set: (value: T | null) => void;
    isProxyOf: (port: Port<any>) => boolean;
}

/**
 * Interface for objects that have a name.
 */
export interface Named {
    /* Return the fully qualified name of this object. */
    getFullyQualifiedName(): string;

    /* Get the name of this object. */
    getName(): string;

    setAlias(name: string): void;
}

//---------------------------------------------------------------------//
// Core Reactor Classes                                                //
//---------------------------------------------------------------------//

class Descendant implements Named {
    
    private alias: string | undefined;

    constructor(protected parent:Descendant | null) {
    
    }

    /**
     * Return a string that identifies this component.
     * The name is a path constructed as TopLevelParentName/.../ParentName/ThisReactorsName
     */
    getFullyQualifiedName(): string {
        var path = "";
        if (this.parent != null) {
            path = this.parent.getFullyQualifiedName();
        }
        if (path != "") {
            path += "/" + this.getName();
        } else {
            path = this.getName();
        }
        return path;
    }

    /* Return a globally unique identifier. */
    public getName(): string {
        var count = 0;
        var suffix = "";
        if (this.alias) {
            return this.alias;
        } else if (this.parent) {
            for (const [key, value] of Object.entries(this.parent)) {
                if (value === this) {
                    return `${key}`;
                }
                // Count instantiations of the same object among entries
                // in order to report unique name for instances among them.
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

    public setAlias(name: string) {
        this.alias = name;
    }
}


/**
 * Generic base class for reactions. The type parameter `T` denotes the
 * type of argument list that the function `react` is applied to when this
 * reaction gets triggered.
 */
export abstract class Reaction<T> implements PrecedenceGraphNode<Priority>, PrioritySetNode<Priority> {
    
    /** Priority derived from this reaction's location in 
     *  the directed acyclic precedence graph. */
    private priority: Priority = Number.MAX_SAFE_INTEGER;

    readonly state = {};

    private next: PrioritySetNode<Priority> | undefined;

    getID(): Reaction<unknown> {
        return this;
    }

    getNext(): PrioritySetNode<Priority> | undefined {
        return this.next;
    }

    setNext(node: PrioritySetNode<Priority> | undefined) {
        this.next = node;
    }

    getDependencies(): [Set<Variable>, Set<Variable>] { 
        var deps:Set<Variable> = new Set();
        var antideps: Set<Variable> = new Set();
        var vars = new Set();
        for (let a of this.args.concat(this.trigs)) {
            if (a instanceof Port) { // FIXME: handle Writers and hierarchical references!
                if (this.parent._isUpstream(a)) {
                    deps.add(a);
                }
                if (this.parent._isDownstream(a)) {
                    antideps.add(a);
                }
            }
        }
        return [deps, antideps];
    }

    getPriority(): Priority {
        return this.priority;
    }

    hasPriorityOver(node: PrioritySetNode<Priority> | undefined): boolean {
        if (node != null && this.getPriority() < node.getPriority()) {
            return true;
        } else {
            return false;
        }
    }

    updateIfDuplicateOf(node:PrioritySetNode<Priority> | undefined) {
        return Object.is(this, node);
    }

    //A reaction defaults to not having a deadline  FIXME: we want the deadline to have access to the same variables
    deadline: null| Deadline = null;

    /** 
     * Construct a new Reaction by passing in a reference to the reactor that contains it,
     * the variables that trigger it, and the arguments passed to the react function.
     * @param state state shared among reactions
     */
    constructor(protected parent:Reactor, public trigs:Variable[], public args:VarList<T>) { // FIXME: make these private and have getters
        this.state = parent.state;
    }

    /**
     * Derived classes must implement this method. Because it is used in a very unusual
     * way -- only by the execution engine, which will apply it to the arguments that
     * were passed into the constructor -- TypeScript will report errors that have to
     * be suppressed by putting //@ts-ignore on the line before the definitions of derived
     * implementations of this method.
     * @param args The arguments to with this function is to be applied.
     */
    public abstract react(...args:VarList<T>): void;

//    private values: Map<Readable<unknown>, unknown> = new Map();

    public doReact() {
        console.log(">>> Reacting >>>" + this.constructor.name);
        this.react.apply(this, this.args);
    }

    /**
     * Setter for reaction deadline. Once a deadline has been set
     * the deadline's timeout will determine whether the reaction's 
     * react function or the deadline's handle function will be invoked.
     * If a deadline has not been set the reaction's react function
     * will always always be invoked. 
     * @param deadline The deadline to attach to this reaction.
     */
    public setDeadline(deadline : Deadline){
        this.deadline = deadline;
    }

    /**
     * Setter for reaction priority. This should
     * be determined by topological sort of reactions.
     * @param priority The priority for this reaction.
     */
    public setPriority(priority: number){
        this.priority = priority;
    }
}

/**
 * The abstract class for a reaction deadline. A deadline is an optional relation
 * between logical time and physical time for a reaction. A reaction possessing a
 * deadline with a timeout of x seconds will invoke the deadline's handler()
 * function instad of its ordinary react() function if the reaction is invoked with 
 * physical time > logical time + timeout.
 */
export abstract class Deadline {

    /**
     * The time after which the deadline miss's handler is invoked.
     */
    private timeout: TimeInterval;

    /**
     * A reference to the reactor this deadline is a part of so it can
     * access reactor state.
     */
    protected state: Reactor;

    /**
     * Getter for timeout.
     */
    public getTimeout(){
        return this.timeout;
    }

    /**
     * This handler function must be overriden by a concrete handler.
     */
    handler(){
        throw new Error("handler function hasn't been defined.")
    }

    /**
     * Deadline constructor.
     * @param state A reference to the state of reactor this deadline is attached to.
     * @param timeout Time after which the deadline has been missed and the deadline
     * miss handler should be invoked.
     */
    constructor(state: Reactor, timeout: TimeInterval){
        this.state = state;
        this.timeout = timeout;
    }
}

/**
 * An event is caused by a timer or a scheduled action. 
 * Each event is tagged with a time instant and may carry a value 
 * of arbitrary type. The tag will determine the event's position
 * with respect to other events in the event queue.
 */
class Event<T> implements PrioritySetNode<TimeInstant> {

    private next: Event<unknown> | undefined;

    /**
     * Constructor for an event.
     * @param trigger The trigger of this event.
     * @param time The time instant when this event occurs.
     * @param value The value associated with this event. 
     * 
     */
    constructor(public trigger: Action<unknown> | Timer, public time: TimeInstant, public value:T) {
    }

    hasPriorityOver(node: PrioritySetNode<TimeInstant> | undefined) {
        if (node) {
            return this.getPriority().isEarlierThan(node.getPriority());
        } else {
            return false;
        }
    }

    updateIfDuplicateOf(node: PrioritySetNode<TimeInstant> | undefined) {
        if (node && node instanceof Event) {
            if (this.trigger === node.trigger && this.time.isSimultaneousWith(node.time)) {
                node.value = this.value; // update the value
                return true;
            } 
        }
        return false;
    }

    // getID(): [Variable, TimeInstant] {
    //     return [this.trigger, this.time];
    // }

    getNext(): Event<unknown> | undefined {
        return this.next;
    }

    setNext(node: Event<unknown> | undefined) {
        this.next = node;
    }
    
    getPriority(): TimeInstant {
        return this.time;
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
export class Action<T> extends Descendant implements Readable<T> {

    origin: Origin;
    minDelay: TimeInterval;
    //name: string;

    // A value is available to any reaction triggered by this action.
    // The value is not directly associated with a timestamp because
    // every action needs a timestamp (for _isPresent()) and only
    // some actions carry values. 
    
    value: T | undefined;
    
    // The most recent time this action was scheduled.
    // Used by the isPresent function to tell if this action
    // has been scheduled for the current logical time.
    
    private timestamp: TimeInstant | undefined;

    public update(e: Event<unknown>) {
        
        if (!e.time.isSimultaneousWith(this.parent.util.getCurrentLogicalTime())) {
            throw new Error("Time of event does not match current logical time.");
        }
        if (e.trigger == this) {
            //@ts-ignore
            this.value = e.value;
            this.timestamp = e.time;
            this.parent.triggerReactions(e);
        } else {
            throw new Error("Attempt to update action using incompatible event.");
        }
    }

    /**
     * Returns true if this action was scheduled for the current
     * logical time. This result is not affected by whether it
     * has a value.
     */
    public isPresent() {
        if(this.timestamp == null){
            // This action has never been scheduled before.
            return false;
        }
        if(this.timestamp.isSimultaneousWith(this.parent.util.getCurrentLogicalTime())){
            return true;
        } else {
            return false;
        }
    }

    public isChildOf(r: Reactor): boolean {
        if (this.parent && this.parent === r) {
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
    public get(): T | null {
        if(this.value && this.isPresent()) {
            return this.value;
        } else {
            return null;
        }
    }

    /** 
     * Construct a new action.
     * @param parent The reactor containing this action.
     * @param origin Optional. If physical, then the hardware clock on the local 
     * platform is used to determine the tag of the resulting event. If logical, 
     * the current logical time (plus one microstep) is used as the offset.
     * @param minDelay Optional. Defaults to 0. Specifies the intrisic delay of
     * any events resulting from scheduling this action.
     */
    constructor(protected parent: Reactor, origin: Origin, minDelay: TimeInterval = new TimeInterval(0)){
        super(parent);
        this.origin = origin;
        this.minDelay = minDelay;
    }

    public toString(){
        return this.getFullyQualifiedName();
    }

    public isSchedulable() {
        return false;
    }

}

export class Startup extends Action<undefined> {

}

export class Scheduler<T> implements Readable<T>, Schedulable<T> {
    
    constructor(private parent:Reactor, private action:Action<T>) {
    
    }
    
    get(): T | null {
        return this.action.get();
    }

    isPresent(): boolean {
        return this.action.isPresent();
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
    schedule(extraDelay: TimeInterval | 0, value?: T) {
        console.log("Scheduling action " + this.action.getName());
        if (!(extraDelay instanceof TimeInterval)) {
            extraDelay = new TimeInterval(0);
        }
        console.log(">>parent: " + this.parent)
        var tag = this.parent.util.getCurrentLogicalTime();
        var delay = this.action.minDelay.add(extraDelay);
        if (this.action.origin == Origin.physical) {
            tag = getCurrentPhysicalTime();
        }
        tag = tag.getLaterTime(delay);
        if (this.action.origin == Origin.logical && !(this.action instanceof Startup)) {
            tag = tag.getMicroStepLater();
        }
        this.parent.util.schedule(new Event(this.action, tag, value));    
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
export class Timer extends Descendant implements Readable<TimeInstant> {
    
    private tag: TimeInstant | undefined;

    get(): TimeInstant | null {
        if (this.tag && this.tag.isSimultaneousWith(this.parent.util.getCurrentLogicalTime())) {
            return this.tag;
        } else {
            return null;
        }
    }

    isPresent(): boolean {
        if (this.get() != null) {
            return true;
        }
        return false;
    }

    period: TimeInterval;
    offset: TimeInterval;

    /**
     * Timer constructor. 
     * @param parent The reactor this timer is attached to.
     * @param offset The interval between the start of execution and the first
     * timer event. Cannot be negative.
     * @param period The interval between rescheduled timer events. If 0, will
     * not reschedule. Cannot be negative.
     */
    constructor(protected parent: Reactor, offset: TimeInterval | 0, period:TimeInterval | 0) {
        super(parent);
        if (!(offset instanceof TimeInterval)) {
            this.offset = new TimeInterval(0);
        } else {
            this.offset = offset;
        }

        if (!(period instanceof TimeInterval)) {
            this.period = new TimeInterval(0);
        } else {
            this.period = period;
        }
    }

    /**
     * Update the current value of this timer in accordance with the given
     * event, and trigger any reactions that list this timer as their trigger.
     * @param e Timestamped event.
     */
    public update(e: Event<unknown>) {
        if (!e.time.isSimultaneousWith(this.parent.util.getCurrentLogicalTime())) {
            throw new Error("Time of event does not match current logical time.");
        }
        if (e.trigger == this) {
            this.tag = e.time;
            this.parent.triggerReactions(e);
        }
    }

    public toString() {
        return "Timer from " + this.parent.getFullyQualifiedName() + " with period: " + this.period + " offset: " + this.offset;
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
export abstract class Reactor extends Descendant {  // FIXME: may create a setter for an alias rather than put a mandatory name in the constructor
    
    public state = {};

    private _triggerMap: Map<Variable, Set<Reaction<unknown>>> = new Map();

    private _dependsOnReactions: Map<Port<unknown>, Set<Reaction<unknown>>> = new Map();

    private _dependentReactions: Map<Port<unknown>, Set<Reaction<unknown>>> = new Map();

    // private _dependsOnPorts: Map<Reaction<unknown>, Set<Port<unknown>>> = new Map();

    // private _dependentOnPorts: Map<Reaction<unknown>, Set<Port<unknown>>> = new Map();

    private _sourcePort: Map<Port<unknown>, Port<unknown>> = new Map();

    private _destinationPorts: Map<Port<unknown>, Set<Port<unknown>>> = new Map();

    private _startupActions: Set<Startup> = new Set();

    private _shutdownActions: Set<Action<unknown>> = new Set();

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
    readonly check = <X extends Variable[]>(...args: X) => args;

    /** Reactions added by the implemented of derived reactor classes. */
    protected _reactions: Reaction<unknown>[] = [];

    private _mutations: Reaction<unknown>[] = []; // FIXME: introduce mutations
    
    public startup: Startup = new Startup(this, Origin.logical);

    public shutdown: Action<undefined> = new Action(this, Origin.logical);

    protected app: App;

    public util: Util;

    protected getWritable<T>(port: Port<T>): Writer<T> {
        return new Writer(port);
    }

    /**
     * Return the set of downstream ports that this reactor connects 
     * to the given port.
     * @param port The port to look up its destinations for.
     */
    public getDestinations(port: Port<unknown>): Set<Port<unknown>> {
        var dests = this._destinationPorts.get(port);
        if (dests) {
            return dests;
        } else {
            return new Set();
        }
    }

    /**
     * Return the set of downstream ports that this reactor connects 
     * to the given port.
     * @param port The port to look up its destinations for.
     */
    public getSource(port: Port<unknown>): Port<unknown>|undefined {
        return this._sourcePort.get(port);
    }

    /**
     * Return the set of reactions within this reactor that are dependent on
     * the given port.
     * @param port The port to look up its depdendent reactions for.
     */
    public getDownstreamReactions(port: Port<unknown>): Set<Reaction<unknown>> {
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
    public getUpstreamReactions(port: Port<unknown>): Set<Reaction<unknown>> {
        var reactions = this._dependsOnReactions.get(port);
        if (reactions) {
            return reactions;
        } else {
            return new Set();
        }
    }


    protected getSchedulable<T>(action: Action<T>) {
        return new Scheduler(this, action);
    }

    protected addReaction<T>(reaction: Reaction<T>) : Reaction<T> {
        // Ensure that arguments are compatible with implementation of react().
        (function<X>(args: VarList<X>, fun: (...args:VarList<X>) => void): void {
        })(reaction.args, reaction.react);
        this._reactions.push(reaction);
        // Stick this reaction into the trigger map to ensure it gets triggered.
        for (let t of reaction.trigs) {
            let s = this._triggerMap.get(t);
            if (s == null) {
                s = new Set();
                this._triggerMap.set(t, s);
            }
            s.add(reaction);
            // Record this trigger as a dependency.
            if (t instanceof Port) {
                this._addDependency(t, reaction);
            } else {
                console.log(">>>>>>>> not a dependency:" + t);
            }
        }

        for (let a of reaction.args) {
            if (a instanceof Port) {
                if (this._isUpstream(a)) {
                    this._addDependency(a, reaction);
                } else if (this._isDownstream(a)) {
                    this._addAntiDependency(a, reaction);
                } else {
                    throw new Error("Encountered argument that is neither a dependency nor an antidependency.");
                }
            }
            // Only necessary if we want to add actions to the dependency graph.
            if (a instanceof Action) {
                // dep
            }
            if (a instanceof Scheduler) {
                // antidep
            }
        }

        return reaction;
    }

    public getPrecedenceGraph(): PrecedenceGraph<Reaction<unknown>> {
        var graph:PrecedenceGraph<Reaction<unknown>> = new PrecedenceGraph();

        for (let r of this._getChildren()) {
            graph.merge(r.getPrecedenceGraph());
        }
        
        let prev: Reaction<unknown> | null = null;
        for (let r of this._reactions) {
            graph.addNode(r);
            // Establish dependencies between reactions
            // depending on their ordering inside the reactor.
            if (prev) {
                graph.addEdge(r, prev);
            }
            var deps = r.getDependencies();
            // look upstream
            for (let d of deps[0]) {
                if (d instanceof Port) {
                    graph.addEdges(r, d.getUpstreamReactions());
                }
            }
            // look downstream
            for (let d of deps[1]) {
                if (d instanceof Port) {
                    graph.addBackEdges(r, d.getDownstreamReactions());
                }
            }
            prev = r;
        }

        return graph;

    }

    private _addDependency(port: Port<unknown>, reaction: Reaction<unknown>): void {
        let s = this._dependentReactions.get(port);
        if (s == null) {
            s = new Set();    
        }
        s.add(reaction);
    }

    private _addAntiDependency(port: Port<unknown>, reaction: Reaction<unknown>): void {
        let s = this._dependsOnReactions.get(port);
        if (s == null) {
            s = new Set();    
        }
        s.add(reaction);
    }

    /**
     * Assign a value to this port at the current logical time.
     * Put the reactions this port triggers on the reaction 
     * queue and recursively invoke this function on all connected output ports.
     * @param value The value to assign to this port.
     */
    public _propagateValue<T>(src: Port<T>): void {
        var value = src.get();
        if (value == null) {
            console.log("Retrieving null value from " + src.getFullyQualifiedName());
            return;
        }
        var reactions = this._triggerMap.get(src);
        // Push triggered reactions onto the reaction queue.
        if (reactions != undefined) {
            for (let r of reactions) {
                this.app._triggerReaction(r);
            }
        } else {
            console.log("No reactions to trigger.")
        }
        // Update all ports that the src is connected to.
        if (this.parent && this.parent instanceof Reactor) {
            var dests = this.parent._destinationPorts.get(src); // FIXME: obtain set of writable object from this map
            if (dests != undefined) {
                for (let d of dests) {
                    // The following is type safe because we're doing
                    // type checks in connect().
                    //@ts-ignore
                    d.update(this.getWritable(d), value);
                }
            } else {
                console.log("No downstream receivers.");
            }
        }
        
    }

    public triggerReactions(e: Event<unknown>) {
        console.log("Triggering reactions sensitive to " + e.trigger);
        let reactions = this._triggerMap.get(e.trigger);
        if (reactions) {
            for (let r of reactions) {
                this.app._triggerReaction(r);
            }
        }
    }

    /**
     * Create a new reactor.
     * @param parent Parent of this reactor.
     */
    constructor(parent: Reactor | null, alias?: string) {
        super(parent);
        if (alias) {
            this.setAlias(alias);
        }
        if (parent != null) {
            this.app = parent.app;
        } else {
            if (this instanceof App) {
                this.app = this;
            } else {
                throw new Error("Cannot instantate reactor without a parent.");
            }
        }
        this.util = this.app.util;

        // Inform parent of this reactor's startup and shutdown action.
        // if (parent != null) {
        //     parent._registerStartupShutdown(this.startup, this.shutdown);
        // }

        // Add default startup reaction.
        var startup = new class<T> extends Reaction<T> {
            // @ts-ignore
            react(): void {
                console.log("*** Performing startup reaction of " + this.parent.getFullyQualifiedName());   
                    // Schedule startup for all contained reactors.
                    this.parent._startupChildren();
                    this.parent._setTimers();
                }
            }(this, [this.startup], []);
        this.addReaction(startup);

        // Add default shutdown reaction.
        var shutdown = new class<T> extends Reaction<T> {
            // @ts-ignore
            react(): void {
                this.parent._unsetTimers();
                // Schedule shutdown for all contained reactors.
            }
        }(this, [this.shutdown], []);
        this.addReaction(shutdown);
    }

    public _startupChildren() {
        for (let r of this._getChildren()) {
            console.log("Propagating startup: " + r.startup);
            this.getSchedulable(r.startup).schedule(0);
        }
    }

    public _shutdownChildren() {
        for (let r of this._getChildren()) {
            console.log("Propagating startup: " + r.startup);
            this.getSchedulable(r.shutdown).schedule(0);
        }
    }

    public isChildOf(parent: Reactor) {
        if (this.parent === parent) {
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
    private _getChildren(): Set<Reactor> {
        let children = new Set<Reactor>();
        for (const [key, value] of Object.entries(this)) {
            // If pointers to other non-child reactors in the hierarchy are not
            // excluded (eg. value != this.parent) this function will loop forever.
            if (value instanceof Reactor && value != this.parent && !(value instanceof App)) {
                // A reactor may not be a child of itself.
                if (value == this){
                    throw new Error("A reactor may not have itself as an attribute." +
                                    " Reactor attributes of a reactor represent children" +
                                    " and a reactor may not be a child of itself");
                }
                children.add(value);
            }
        }
        return children;
    }

    protected mutations = new class implements Mutations {
        constructor(public superThis: Reactor) {
        }
        public connect<D, S extends D>(src:Port<S>, dst:Port<D>) {
            return this.superThis._connect(src, dst);
        }
    }(this);

    // public _registerStartupShutdown(startup: Startup, shutdown: Action<unknown>) {
    //     // FIXME: do hierarchy check to ensure that this reactors should have access to these actions.
    //     this._startupActions.add(startup);
    //     this._shutdownActions.add(shutdown);
    // }
    
    /**
     * Returns the set of reactions owned by this reactor.
     */
    public _getReactions(): Set<Reaction<unknown>> {
        var set:Set<Reaction<unknown>> = new Set();
        for (let entry of this._reactions) {
            set.add(entry);
        }
        return set;
    }

    public _isDownstream(arg: Port<unknown>) {
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

    public _isUpstream(arg: Port<unknown>) {
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
    }

    // public _inScope(var: Variable<any>): boolean {
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
    public canConnect<D, S extends D>(src: Port<S>, dst: Port<D>): boolean {
        // 1. Rule out self loops. 
        //   - (including trivial ones)
        if (src === dst) {
            return false;
        }

        // FIXME: check the local dependency graph to figure out whether this
        // change introduces zero-delay feedback.
        
        // 2. Rule out write conflicts.
        //   - (between reactors)
        if (this._sourcePort.get(dst) != undefined) {
            return false;
        }

        //   - between reactors and reactions 
        // (NOTE: check also needs to happen in addReaction)
        var antideps = this._dependsOnReactions.get(dst);
        if (antideps != undefined && antideps.size > 0) {
            return false;
        }

        // 3. Assure that the scoping rules are adhered to.
        if (src instanceof OutPort) {
            if (dst instanceof InPort){ 
                // OUT to IN
                if(src.isGrandChildOf(this) && dst.isGrandChildOf(this)) {
                    return true;
                } else {
                    return false;
                }
            } else {
                // OUT to OUT
                if(src.isGrandChildOf(this) && dst.isChildOf(this)) {
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            if (src === dst){
                return false;
            }
            if (dst instanceof InPort){
                // IN to IN
                if(src.isChildOf(this) && dst.isGrandChildOf(this)){
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

    protected _connect<D, S extends D>(src:Port<S>, dst:Port<D>) {
        //***********
        if (this.canConnect(src, dst)) {
            console.log("connecting " + src + " and " + dst);
            let dests = this._destinationPorts.get(src);
            if (dests == null) {
                dests = new Set();
            }
            dests.add(dst);
            this._destinationPorts.set(src, dests);
            this._sourcePort.set(dst, src);
        } else {
            console.log("ERROR connecting.");
        }
    }

    protected _disconnect(src:Port<unknown>, dst: Port<unknown>) {
        console.log("disconnecting " + src + " and " + dst);
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
    public _setTimers(): void {
        console.log("Setting timers for: " + this);
        let timers = new Set<Timer>();
        for (const [k, v] of Object.entries(this)) {
            if (v instanceof Timer) {
                this.app._setTimer(v);
            }
        }
    }

    /**
     * Unset all the timers of this reactor.
     */
    public _unsetTimers(): void {
        // console.log("Getting timers for: " + this)
        let timers = new Set<Timer>();
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
    public _getPorts(): Set<Port<any>>{
        // console.log("Getting ports for: " + this)
        let ports = new Set<Port<any>>();
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
    public _getActions(): Set<Action<any>>{
        // console.log("Getting actions for: " + this)
        let actions = new Set<Action<any>>();
        for (const [key, value] of Object.entries(this)) {
            if (value instanceof Action) {
                actions.add(value);
            }
        }
        return actions;
    }




    //A reactor's priority represents its order in the topological sort.
    //The default value of -1 indicates a priority has not been set.
    _priority: number = -1;

    //FIXME: assign in constructor?
 
    toString(): string {
        return this.getFullyQualifiedName();
    }

    /**
     * Recursively sets the app attribute for this reactor and all contained reactors to app.
     * @param app The app for this and all contained reactors.
     */
    // public _setApp(app: App){
    //     // console.log("Starting _setApp for: " + this._getFullyQualifiedName());
    //     console.log("Setting app for: " + this);
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
    public _checkAllParents(parent: Reactor | null){
        if(this.parent != parent) throw new Error("The parent property for " + this
            + " does not match the reactor hierarchy.");
        
        // FIXME: check that there exist no copies?
        // This might be difficult...

        let children = this._getChildren();
        for(let child of children) {
            child._checkAllParents(this);
        }

        // Ports have their parent set in constructor, so verify this was done correctly.
        let ports = this._getPorts();
        for(let port of ports){
            if(!port.isChildOf(this)){
                throw new Error("A port has been incorrectly constructed as an attribute of " +
                                "a different reactor than the parent it was given in its constructor: "
                                + port);
            }
        }

        let actions = this._getActions();
        for(let action of actions){
            if (!action.isChildOf(this)) throw new Error("The parent property for " + action
            + " does not match the reactor hierarchy.");
        }

    }

}

export abstract class Port<T> extends Descendant implements Readable<T> {
    
    /** The time stamp associated with this port's value. */  
    protected tag: TimeInstant | undefined;

    /** The value associated with this port. */  
    protected value: T | null = null;
    
    // public _connectedSinkPorts: Set<Port<unknown>> = new Set<Port<T>>(); // FIXME: change this into a private map hosted in the reactor
    // public _connectedSourcePort: Port<T>| null = null; // FIXME: change this into a private map hosted in the reactor

    /**
     * Return the transitive closure of reactions dependent on this port.
     */
    public getDownstreamReactions(): Set<Reaction<unknown>> {
        var reactions: Set<Reaction<unknown>> = new Set();
        for (let d of this.parent.getDestinations(this)) {
            reactions = new Set([...reactions, ...d.getDownstreamReactions()]);
        }
        reactions = new Set([...reactions, ...this.parent.getDownstreamReactions(this)]);
        return reactions;
    }


    /**
     * Return the transitive closure of reactions dependent on this port.
     */
    public getUpstreamReactions(): Set<Reaction<unknown>> {
        var reactions: Set<Reaction<unknown>> = new Set();
        var source = this.parent.getSource(this);
        if (source) {
            reactions = new Set([...reactions, ...source.getUpstreamReactions()]);
        }
        reactions = new Set([...reactions, ...this.parent.getUpstreamReactions(this)]); // FIXME
        return reactions;
    }

    public isChildOf(r: Reactor): boolean {
        if (this.parent && this.parent === r) {
            return true;
        }
        return false;
    }

    public isGrandChildOf(r: Reactor): boolean {
        if (this.parent && this.parent.isChildOf(r)) {
            return true;
        }
        return false;
    }    

    /**
     * Returns true if the connected port's value has been set.
     * an output port or an input port with a value set at the current logical time
     * Returns false otherwise
     */
    public isPresent(){
        if(this.value 
            && this.tag 
            && this.tag.isSimultaneousWith(this.parent.util.getCurrentLogicalTime())) {
            return true;
        } else {
            return false;
        }
    }

    public update<X>(writer: Writable<X>, value: X) { 
        if (writer.isProxyOf(this)) {
            // Only update the value if the proxy has a reference
            // to this port. If it does, the type variables must
            // match; no further checks are needed.
            log.debug("Updating value of " + this.getFullyQualifiedName());
            //@ts-ignore
            this.value = value;
            log.debug(">> parent: " + this.parent);
            this.tag = this.parent.util.getCurrentLogicalTime();
            this.parent._propagateValue(this); // FIXME: should this be a utility function?
        } else {
            log.warn("WARNING: port update denied.");
        }
    }

    /**
     * Obtains the value set to this port. Values are either set directly by calling set()
     * on this port, or indirectly by calling set() on a connected upstream port.
     * Will return null if the connected output did not have its value set at the current
     * logical time.
     */
    public get(): T | null {
        if (this.value && this.isPresent()){
            return this.value;
        } else {
            return null;
        }
    }

    /**
     * Create a new port on the given reactor.
     * @param parent 
     */
    constructor(protected parent: Reactor) {
        super(parent);
    }

    toString(): string {
        return this.getFullyQualifiedName();
    }
}


export class OutPort<T> extends Port<T> implements Port<T> {

    toString(): string {
        return this.getFullyQualifiedName();
    }

}

export class InPort<T> extends Port<T> {

    toString(): string {
        return this.getFullyQualifiedName();
    }

}

class Writer<T> implements Writable<T> { // NOTE: don't export this class!

    constructor(private port: Port<T>) {
    }
    
    public isPresent() {
        return this.port.isPresent();
    }

    /**
    * Write a value and recursively transmit it to connected ports, which may
    * trigger downstream reactions. No action is taken if the given value is
    * null.
    * @param value The value to be written.
    */
    public set(value: T | null):void {
        console.log("set() has been called on " + this.port.getFullyQualifiedName());
        if (value != null) {
            this.port.update(this, value);
        }
    }

    public get(): T | null {
        return this.port.get();
    }

    public isProxyOf(port: Port<T>): boolean {
        if (this.port === port) {
            return true;
        }
        return false;
    }
}

class EventQueue extends PrioritySet<TimeInstant> {

    public push(event: Event<unknown>) {
        return super.push(event);
    }

    public pop(): Event<unknown> | undefined { 
        return super.pop() as Event<unknown>;
    }

    public peek(): Event<unknown> | undefined {
        return super.peek() as Event<unknown>;
    }
}

class ReactionQueue extends PrioritySet<Priority> {

    public push(reaction: Reaction<unknown>) {
        return super.push(reaction);
    }

    public pop(): Reaction<unknown> { 
        return super.pop() as Reaction<unknown>;
    }

    public peek(): Reaction<unknown> {
        return super.peek() as Reaction<unknown>;
    }
}

export class App extends Reactor { // Perhaps make this an abstract class, like reactor; omit the name parameter.
    
    // FIXME: add some logging facility here
    util = new class implements Util {
        constructor(private app: App) {
        
        }

        getCurrentLogicalTime(): TimeInstant {
            return this.app._currentLogicalTime;
        }

        getCurrentPhysicalTime(): TimeInstant {
            return getCurrentPhysicalTime();
        }

        getElapsedLogicalTime(): TimeInterval {
            return this.app._currentLogicalTime.getTimeDifference(this.app._startOfExecution);
        }

        getElapsedPhysicalTime(): TimeInterval {
            return getCurrentPhysicalTime().getTimeDifference(this.app._startOfExecution);
        }

        success(): void {
            return this.app.success();
        }

        failure(): void {
            return this.app.failure();
        }

        public schedule(e: Event<any>) {
            return this.app.schedule(e);
        }
        
    }(this);


    /**
     * The current time, made available so actions may be scheduled relative to it.
     */
    private _currentLogicalTime: TimeInstant;

    private alarm = new NanoTimer();

    /**
     * Priority set that keeps track of scheduled events.
     */
    private _eventQ = new EventQueue();

    /**
     * If not null, finish execution with success, this time interval after
     * the start of execution.
     */
    private _executionTimeout: TimeInterval | null = null;

    /**
     * 
     */
    private _endOfExecution: TimeInstant | undefined;

    /**
     * The physical time when execution began relative to January 1, 1970 00:00:00 UTC.
     * Initialized in start().
     */
    private _startOfExecution: TimeInstant;

    /**
     * Indicates whether the program should continue running once the event 
     * queue is empty.
     */
    private _keepAlive = false;

    /**
     * Priority set that keeps track of reactions at the current logical time.
     */
    private _reactionQ = new ReactionQueue();

    /**
     * Report a timer to the app so that it gets scheduled.
     * @param timer The timer to report to the app.
     */
    public _setTimer(timer: Timer) {
        console.log(">>>>>>>>>>>>>>>>>>>>>>>>Setting timer: " + timer);
        this.schedule(new Event(timer, 
            this.util.getCurrentLogicalTime().getLaterTime(timer.offset), 
            null));
    }

    /**
     * Report a timer to the app so that it gets unscheduled.
     * @param timer The timer to report to the app.
     */
    public _unsetTimer(timer: Timer) {
        // push a new event onto the event queue
        // FIXME: we could either set the timer to 'inactive' to tell the 
        // scheduler to ignore future event and prevent it from rescheduling any.
        // The problem with this approach is that if, for some reason, a timer would get
        // reactivated, it could start seeing events that were scheduled prior to its
        // becoming inactive. Alternatively, we could remove the event from the queue, 
        // but we'd have to add functionality for this.
    }

    //private heartbeat: Timer = new Timer(this, 0, new TimeInterval(5));
    private snooze: Action<TimeInstant> = new Action(this, Origin.logical, new TimeInterval(5, 0));

    /**
     * Create a new top-level reactor.
     * @param executionTimeout Optional parameter to let the execution of the app time out.
     * @param success Optional callback to be used to indicate a successful execution.
     * @param failure Optional callback to be used to indicate a failed execution.
     */
    constructor(executionTimeout: TimeInterval | null = null, public success: ()=> void = () => {}, public failure: ()=>void = () => {}) {
        super(null);
        App.instances.add(this);
        this._executionTimeout = executionTimeout;
        // NOTE: these will be reset properly during startup.
        this._currentLogicalTime = new TimeInstant(new TimeInterval(0), 0);
        this._startOfExecution = this._currentLogicalTime;
    }

    static instances: Set<App> = new Set(); // FIXME: we have to remove the instance from the set when we're done with it, or this will create a memory leak.

    getName(): string {
        var alias = super.getName();
        var count = 0;
        var suffix = "";
        if (alias == this.constructor.name) {
            for (let a of App.instances) {
                if (a !== this && alias === a.constructor.name) {
                    count++;
                }
            }
        }
        if (count > 0) {
            suffix = "(" + count + ")";
        }
        return alias + suffix;
    }

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
    private _next() {
        console.log("New invocation of next().");
        console.log("Logical time: " + this.util.getCurrentLogicalTime());
        console.log("Physical time: " + getCurrentPhysicalTime());

        // Terminate with success if timeout has been reached.
        if (this._endOfExecution && this._endOfExecution.isEarlierThan(getCurrentPhysicalTime())) {
            console.log("Execution timeout reached. Terminating runtime with success.");
            this.success();
            return;
        }

        var nextEvent = this._eventQ.peek();

        if (nextEvent == undefined) {
            console.log("Empty event queue.");
            this.suspendOrTerminate();
            return;
        } else if (getCurrentPhysicalTime().isEarlierThan(nextEvent.time)) {
            // Simply return if it is too early to handle the next event;
            // Next will be invoked when the alarm goes off.
            console.log("Too early to handle next event.");
            return;
        } else {
            if (nextEvent.trigger === this.snooze) {
                console.log("Woken up after suspend.");
            }
            this._currentLogicalTime = nextEvent.time;
            console.log("Processing events. Logical time elapsed: " + this.util.getElapsedLogicalTime());
            while (nextEvent != null && nextEvent.time.isSimultaneousWith(this._currentLogicalTime)) {
                this._eventQ.pop();
                // Handle timers.
                if (nextEvent.trigger instanceof Timer) {
                    if (!nextEvent.trigger.period.isZero()) {
                        console.log("Rescheduling timer " + nextEvent.trigger);
                        // reschedule
                        this.schedule(new Event(nextEvent.trigger, 
                            this._currentLogicalTime.getLaterTime(nextEvent.trigger.period), 
                            null));
                    }
                }
                // Load reactions onto the reaction queue.
                nextEvent.trigger.update(nextEvent);
                // Look at the next event on the queue.
                nextEvent = this._eventQ.peek();
            }

            while(this._reactionQ.size() > 0) {
                // Test if this reaction has a deadline which has been violated.
                // This is the case if the reaction has a registered deadline and
                // logical time + timeout < physical time
                // FIXME: temporarily disabled deadlines
                // if(r.deadline && compareNumericTimeIntervals( 
                //         numericTimeSum(this._currentLogicalTime[0], timeIntervalToNumeric(r.deadline.getTimeout())),
                //         currentPhysicalTime)){
                //     console.log("handling deadline violation");
                //     r.deadline.handler();
                // } else {
                this._reactionQ.pop().doReact();
                // }
            }
            nextEvent = this._eventQ.peek();
            if (nextEvent != null) {
                // Set an alarm for the next invocation of _next.
                this.setAlarm(nextEvent);
            } else {
                // Or suspend/terminate.
                console.log("Empty event queue.")
                this.suspendOrTerminate();
            }
        }
    }

    private suspendOrTerminate() {
        if (this._keepAlive) {
            // Snooze if event queue is empty and keepalive is true.
            console.log("Going to sleep.");
            this.getSchedulable(this.snooze).schedule(0, this._currentLogicalTime);
            return;
        } else {
            this.success();
            return;
        }
    }

    /**
     * Public method to push events on the event queue. 
     * @param e Prioritized event to push onto the event queue.
     */
    public schedule(e: Event<any>) {
        let head = this._eventQ.peek();
        this._eventQ.push(e);
        if (head == undefined || e.time.isEarlierThan(head.time)) {
            // Reset the timer if this new event is earlier than 
            // whatever was found at the head of the event queue.
            this.setAlarm(e);
        }        
    }

    public setAlarm(e : Event<unknown>) {
        let physicalTime = getCurrentPhysicalTime();

        // Set alarm for end of execution or next event, whichever comes first.
        if (this._endOfExecution && this._endOfExecution.isEarlierThan(e.time)) {
            let timeout = e.time.getTimeDifference(this._endOfExecution);
            this.alarm.setTimeout(this._next.bind(this), '', timeout.getNanoTime());
        } else if (physicalTime.isEarlierThan(e.time)) {
            let timeout = e.time.getTimeDifference(physicalTime);
            this.alarm.setTimeout(this._next.bind(this), '', timeout.getNanoTime());
        } else {
            this.alarm.setTimeout(this._next.bind(this), '', "0n");
        }
    }

    /**
     * Public method to push reaction on the reaction queue. 
     * @param e Prioritized reaction to push onto the reaction queue.
     */
    public _triggerReaction(r: Reaction<unknown>){  
        this._reactionQ.push(r);
    }

    public _start():void {
        // Recursively check the parent attribute for this and all contained reactors and
        // and components, i.e. ports, actions, and timers have been set correctly.
        this._checkAllParents(null);
        // Recursively set the app attribute for this and all contained reactors to this.
        // Set reactions using a topological sort of the dependency graph.
        var apg = this.getPrecedenceGraph();
        if (apg.updatePriorities()) {
            console.log("No cycles.");
        } else {
            throw new Error("Cycle in reaction graph.");
        }
        console.log(apg.toString());
        this._startOfExecution = getCurrentPhysicalTime();
        this._currentLogicalTime = this._startOfExecution;
        if(this._executionTimeout != null) {
            this._endOfExecution = this._startOfExecution.getLaterTime(this._executionTimeout);
            console.log("Execution timeout: " + this._executionTimeout);
        }
        // Set in motion the execution of this program.
        this.getSchedulable(this.startup).schedule(0);
    }

}

