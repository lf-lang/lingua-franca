/**
 * Core of the reactor runtime.
 * 
 * @author Marten Lohstroh (marten@berkeley.edu),
 * @author Matt Weber (matt.weber@berkeley.edu)
 */

import {PrecedenceGraph, PrecedenceGraphNode, PrioritySetNode, PrioritySet} from './util';
import "./time";
import { TimeInterval, TimeInstant, compareTimeInstants, TimelineClass, TimestampedValue, NumericTimeInterval, microtimeToNumeric, compareNumericTimeIntervals, timeIntervalIsZero, timeIntervalToNumeric, timeInstantsAreEqual, numericTimeSum, numericTimeMultiple, numericTimeDifference } from './time';
// import * as globals from './globals'

//---------------------------------------------------------------------//
// Modules                                                             //
//---------------------------------------------------------------------//

//Must first declare require function so compiler doesn't complain
declare function require(name:string);

const microtime = require("microtime");
const NanoTimer = require('nanotimer');

//---------------------------------------------------------------------//
// Types                                                               //
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
// Runtime Functions                                                   //
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
// Interfaces                                                          //
//---------------------------------------------------------------------//

/**
 * A Trigger is something which can cause an Event: a Timer, or an action.
 * Technically, inputs and output from contained reactors can also cause events,
 * but they can just directly put reactions on the reaction queue.
 * Reactions may register themselves as triggered by a Trigger. 
 */

export interface Trigger{}

// FIXME: I don't think we need this?
export interface Writable<T> {
    set: (value: T | null) => void;
}

// FIXME: I don't think we need this.
export interface Readable<T> {
    get: () => T | null;
    isPresent: () => boolean;
}


/**
 * For objects that have a name.
 */
export interface Named {
    /* Return the fully qualified name of this object. */
    _getFullyQualifiedName(): string;

    /* Get the name of this object. */
    _getName(): string;
}

//FIXME: remove this?
/**
 * For (re)nameable objects.
 */
export interface Nameable extends Named {
 
    /* Set the name of this object. */
    _setName(name: string):void;
}

export interface Mutation {
    
    parent:Set<Reactor>;
    reactions:Array<Reaction>;

    new(parent: Set<Reactor>, reactions:Array<Reaction>):Mutation;
}

//---------------------------------------------------------------------//
// Core Reactor Classes                                                //
//---------------------------------------------------------------------//

/**
 * The reaction abstract class.
 * A concrete reaction class should extend Reaction, and implement a react function.
 */
export abstract class Reaction{

    //FIXME: for now state is the entire this for the parent reactor. This should be changed
    //to a custom data structure with only the state relevant to a reaction.
    state: Reactor;

    //Contains the timers, actions, and inputs that trigger this reaction.
    triggers: Array<Trigger>;
    priority: number;
    triggeringActions: Set<Action<any>>;

    // The dependencies for this reaction.
    uses: Array<InPort<any>>;
    // The antidependencies for this reaction.
    effects: Array<OutPort<any> | Action<any>> 

    //A reaction defaults to not having a deadline  
    deadline: null| Deadline = null;

    /**
     * Register this reaction's triggers with the app.
     * This step can't be handled in the constructor because
     * the app for this reaction is not known at that time.
     * The app will call this function later as part of its setup process.
     */
    public register(){
        // console.log("Before register reaction");
        this.state._app._triggerMap.registerReaction(this);
        // console.log("After register reaction");
    }

    public toString(){
        let message = "Reaction with triggers: [ ";
        for(let trigger of this.triggers){
            message += trigger.toString() + ", "
        }
        message += "] uses: [ ";
        for(let use of this.uses){
            message += use.toString() + ", "
        }
        message += "] effects: [ ";
        for(let effect of this.effects){
            message += effect.toString() + ", "
        }
        message += "]";
        return  message;
    }

    /**
     * Reactor constructor: FIXME
     * @param state FIXME: should probably be named parent
     * @param triggers 
     * @param priority 
     */
    constructor(state: Reactor, triggers: Array<Trigger>,
                 uses: Array<InPort<any>> , effects: Array<OutPort<any> | Action<any>> ){
        this.triggers = triggers;
        this.state = state;
        this.uses = uses;
        this.effects = effects;
    }

    /**
     * This react function must be overridden by a concrete reaction.
     */
    public abstract react(): void;

    /**
     * More concise way to get logical time in a reaction.
     */
    public _getcurrentlogicaltime(){
        return this.state._app._getcurrentlogicaltime();
    }

    /**
     * Getter function for dependencies (uses)
     */
    public getUses(){
        return this.uses;
    }

    /**
     * Getter function for anti dependencies (effects)
     */
    public getEffects(){
        return this.effects;
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
export abstract class Deadline{

    /**
     * The time after which the deadline miss's handler is invoked.
     */
    private timeout: TimeInterval;

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
     * @param timeout Time after which the deadline has been missed and the deadline
     * miss handler should be invoked.
     */
    constructor(timeout: TimeInterval){
        this.timeout = timeout;
    }
}


/**
 * A prioritized reaction wraps a Reaction with a priority and precedence
 * and may be inserted into the reaction queue.
 * The priority of a reaction depends on the priority of its reactor, which is
 * determined by a topological sort of reactors.
 */
export class PrioritizedReaction implements PrecedenceGraphNode,
 PrioritySetNode<number,number>{

    r: Reaction;

    //Precedence graph node attributes
    _id: number;
    _next: PrioritySetNode<number,number> | null;
    _priority: number;

    constructor(r: Reaction, id: number) {
        this.r = r;
        this._id = id;
        this._next = null;
        this._priority = r.priority;
    }
    
    hasPrecedenceOver(node: PrioritySetNode<number,number>) {
        if (this._priority < node._priority) {
            return true;
        } else {
            return false;
        }
    }    
}

/**
 * 
 * An event is caused by a timer, or caused by a scheduled action (an internal event)
 * It occurs at a particular time instant and carries an arbitrary data type as value.
 * An Event has a priority and precedence so it may be inserted into the event queue.
 * The priority of an Event is determined by its time. An Event with an earlier time
 * than another has precedence.
 */
export class Event<T> implements 
 PrioritySetNode<number,TimeInstant>{

    // Event attributes
    public time: TimeInstant;
    public cause: Trigger;
    
    // Associating values with events makes it possible
    // to ensure actions take on the value assigned the last
    // time they are scheduled.
    public value: T;

    // Precedence graph node attributes
    public _id: number;
    public _next: PrioritySetNode<number,TimeInstant> | null;
    public _priority: TimeInstant;

    /**
     * Constructor for an event.
     * @param cause The trigger which "caused" this event to occur.
     * @param time The time instant when this event occurs.
     * @param value The value associated with this event. 
     * @param id A unique identifier for this event. This should probably
     * be obtained from app.getEventID().
     * 
     */
    constructor(cause: Trigger, time: TimeInstant, value:T, id: number){
        this.time = time;
        this.cause = cause;
        this.value = value;
        this._id = id;
        this._next = null;
        this._priority = time;
    }

    hasPrecedenceOver(node: PrioritySetNode<number,TimeInstant>) {
        return compareTimeInstants(this._priority, node._priority);
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
export class Action<T> implements Trigger, Readable<T> {

    // The constructor for a reactor sets this attribute for each
    // of its attached timers.
    parent: Reactor;

    timeType: TimelineClass;
    minDelay: TimeInterval;
    name: string;

    // A value is available to any reaction triggered by this action.
    // The value is not directly associated with a timestamp because
    // every action needs a timestamp (for _isPresent()) and only
    // some actions carry values. 
    
    //FIXME: make private?
    value: T;
    
    // The most recent time this action was scheduled.
    // Used by the isPresent function to tell if this action
    // has been scheduled for the current logical time.
    
    //FIXME: make private?
    timestamp: TimeInstant | null;

    /**
     * Returns true if this action was scheduled for the current
     * logical time. This result is not affected by whether it
     * has a value.
     */
    public isPresent(){
        if(this.timestamp == null){
            // This action has never been scheduled before.
            return false;
        }
        if(timeInstantsAreEqual(this.timestamp, this.parent._app._getcurrentlogicaltime())){
            return true;
        } else {
            return false;
        }
    }

    /**
     * Called on an action within a reaction to acquire the action's value.
     * The value for an action is set by a scheduled action event, and is only
     * present for reactions executing at that logical time. When logical time
     * advances, that previously available value is now unavailable.
     * If the action was scheduled with no value, this function returns null.
     */
    public get(): T | null{
        if(this.value && this.isPresent()){
            return this.value;
        } else {
            return null;
        }
    }

    /** 
     * Action Constructor
     * @param parent The Reactor containing this action.
     * @param timeType Optional. Defaults to physical. If physical,
     *  then the physical clock on the local platform is used to assign a timestamp
     *  to the action when it is enabled. If logical, the current physical time is
     *  ignored and the timestamp of the action is the current logical time (plus
     *  one microstep) or, if a minimum delay is given and is greater than zero,
     *  the current logical time plus the minimum delay.
     * @param minDelay Optional. Defaults to 0. If a minDelay is given, then it 
     *  specifies a minimum delay, the minimum logical time that must elapse between
     *  when the action is enabled and when it triggers. If the delay parameter to the
     *  schedule function and the mindelay parameter are both zero and the physical
     *  keyword is not given, then the action is timestamped one microstep later.
     */
    constructor(parent: Reactor, timeType: TimelineClass = TimelineClass.physical, minDelay: TimeInterval = 0){
        this.parent = parent;
        this.timeType = timeType;
        this.minDelay = minDelay;
        this.name = name;
    }

    /**
     * Schedule this action. An event for this action will be
     * created and pushed onto the event queue. If the same action
     * is scheduled multiple times for the same logical time, the value
     * associated with the last invocation of the this function determines
     * the value attached to the action at that logical time.
     * @param delay The time difference between now and the future when 
     * this action should occur. 
     * @param value An optional value to be attached to this action.
     * The value will be available to reactions depending on this action.
     */
    schedule(delay: TimeInterval, value?: T){
        console.log("Scheduling action.");
        if(delay === null){
            throw new Error("Cannot schedule an action with a null delay");
        }

        let timestamp: TimeInstant;
        let wallTime: NumericTimeInterval; 

        //FIXME: I'm not convinced I understand the spec so,
        //Probably something wrong in one of these cases...
        if(this.timeType == TimelineClass.physical){
            //physical
            wallTime = microtimeToNumeric(microtime.now());
            if(compareNumericTimeIntervals( this.parent._app._getcurrentlogicaltime()[0], wallTime )){
                timestamp = [this.parent._app._getcurrentlogicaltime()[0], this.parent._app._getcurrentlogicaltime()[1] + 1 ];
            } else {
                timestamp = [wallTime, 0 ];
            }
        } else {
            //logical
            if( timeIntervalIsZero(this.minDelay) && timeIntervalIsZero(delay)) {
                timestamp = [this.parent._app._getcurrentlogicaltime()[0], this.parent._app._getcurrentlogicaltime()[1] + 1 ];
            } else {
                //Take min of minDelay and delay
                let numericMinDelay = timeIntervalToNumeric(this.minDelay);
                let numericDelay = timeIntervalToNumeric(delay);
                let actionTime: NumericTimeInterval;
                if(compareNumericTimeIntervals(numericMinDelay, numericDelay )){
                    actionTime = numericMinDelay;
                } else{
                    actionTime = numericDelay;
                }
                timestamp = [actionTime, this.parent._app._getcurrentlogicaltime()[1]];
            }
        }

        let actionEvent = new Event(this, timestamp, value, this.parent._app._geteventID());
        // let actionPriEvent = new PrioritizedEvent(actionEvent, this.parent.app.getEventID());
        this.parent._app._scheduleEvent(actionEvent);    
    }

    /**
     * Setter method for an action's parent attribute.
     * @param parent The reactor this action is attached to.
     */
    public _setParent(parent: Reactor){
        this.parent = parent;
    }

    /**
     * Getter method for an action's parent attribute.
     */
    public _getParent(){
        return this.parent;
    }

    public toString(){
        return "Action of " + this.parent;
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
export class Timer{
    
    // The reactor this timer is attached to.
    parent: Reactor;

    //For reference, the type of a TimeInterval is defined as:
    //TimeInterval = null | [number, TimeUnit] | 0;
    period: TimeInterval;
    offset: TimeInterval;

    //A timer is only triggered by itself.
    triggers: Array<Trigger> = new Array();

    //Private variables used to keep track of rescheduling
    _timerFirings: number = 0;
    _offsetFromStartingTime: NumericTimeInterval;

    //The setup function is used by an app to create the first timer event.
    //It must be called before reschedule, or else _offsetFromStartingTime will
    //not be set.
    setup(){
        if(this.offset === 0 || this.offset[0] >= 0){
            
            let numericOffset = timeIntervalToNumeric(this.offset);
            this._offsetFromStartingTime =  numericTimeSum( numericOffset, this.parent._app._getStartingWallTime() );
            let timerInitInstant: TimeInstant = [this._offsetFromStartingTime, 0];
            let timerInitEvent: Event<null> = new Event(this, timerInitInstant, null, this.parent._app._geteventID());

            this.parent._app._scheduleEvent(timerInitEvent);

            console.log("Scheduled timer init for timer with period " + this.period + " at " + timerInitInstant);
        } else {
            throw new Error("Cannot setup a timer with a negative offset.");
        }
    }

    /**
     * The reschedule function for a timer schedules the next timer event using the period.
     * A period of 0 indicates the timer should not be scheduled again.
     * Note that rescheduling is based on a multiple of the period and does not "slip"
     * if the last scheduling happened late.
     */
    reschedule() {
        this._timerFirings++;
        if((this.period === 0 || this.period[0] >= 0)){
            if( !timeIntervalIsZero(this.period)){

                let numericPeriod = timeIntervalToNumeric(this.period);
                let nextLogicalTime: NumericTimeInterval = numericTimeSum(this._offsetFromStartingTime, 
                                    numericTimeMultiple(numericPeriod , this._timerFirings) ); 
                let nextTimerInstant: TimeInstant = [nextLogicalTime, 0];
                let nextTimerEvent: Event<null> = new Event(this, nextTimerInstant, null, this.parent._app._geteventID());
                // console.log("In reschedule, this.parent : " + this.parent);
                this.parent._app._scheduleEvent(nextTimerEvent);

                console.log("Scheduling next event for timer with period " + this.period + " for time: " + nextTimerInstant);
            }

        } else {
            throw new Error("Cannot reschedule a timer with a negative period.");
        }
    };

    /**
     * Timer constructor. 
     * @param parent The reactor this timer is attached to.
     * @param offset The interval between the start of execution and the first
     * timer event. Cannot be negative.
     * @param period The interval between rescheduled timer events. If 0, will
     * not reschedule. Cannot be negative.
     */
    constructor(parent: Reactor, offset:TimeInterval, period:TimeInterval) {
        this.parent = parent;
        this.period = period;
        this.offset = offset;

        if(offset[0] < 0){
            throw new Error("A timer offset may not be negative.");
        }

        if(period[0] < 0){
            throw new Error("A timer period may not be negative.");
        }
    }

    /**
     * Setter method for a timer's parent attribute.
     * @param parent The reactor containing this timer.
     */
    public _setParent(parent: Reactor){
        this.parent = parent;
        // console.log("Setting parent for " + this);
    }

    /**
     * Getter method for a timer's parent attribute.
     */
    public _getParent(){
        return this.parent;
    }

    public toString(){
        return "Timer with period: " + this.period + " offset: " + this.offset;
    }

    /**
     * FIXME: not implemented yet. Do we need this?
     * @param period 
     */
    adjustPeriod(period: TimeInterval):void {   
        // FIXME
    }
}

/**
 * A reactor is a software component that reacts to input events,
 * timer events, and internal events. It has private state variables
 * that are not visible to any other reactor. Its reactions can
 * consist of altering its own state, sending messages to other
 * reactors, or affecting the environment through some kind of
 * actuation or side effect.
 */
export abstract class Reactor implements Nameable{

    // FIXME _mutations currently aren't used for anything.
    private _mutations:Array<
            [   // triggers, mutation, mutation arguments
                Array<Trigger>, Mutation, any
            ]
    >;
    
    //FIXME: make this private. Right now this would break tests.
    _reactions:Array<Reaction> = new Array<Reaction>();
    
    private _myName: string;
    private _myIndex: number | null;

    protected _parent: Reactor|null = null;
    
    public _app:App;

    /**
     * Obtain the set of this reactor's child reactors.
     * Watch out for cycles!
     * This function ignores reactor attributes which reference
     * this reactor's parent and this reactor's app.
     * It is an error for a reactor to be a child of itself.
     */
    public _getChildren(): Set<Reactor> {
        let children = new Set<Reactor>();
        for (const [key, value] of Object.entries(this)) {
            // If pointers to other non-child reactors in the hierarchy are not
            // excluded (eg. value != this.parent) this function will loop forever.
            if (value instanceof Reactor && value != this._getParent() && value != this._app) {
                // A reactor may not be a child of itself.
                if(value == this){
                    throw new Error("A reactor may not have itself as an attribute." +
                                    " Reactor attributes of a reactor represent children" +
                                    " and a reactor may not be a child of itself");
                }
                children.add(value);
            }
        }
        return children;

    }

    /**
     * Returns the set of reactions owned by this reactor.
     */
    public _getReactions(): Set<Reaction> {
        console.log("Getting reactions for: " + this);
        let reactions = new Set<Reaction>();
        for (const [key, value] of Object.entries(this)) {
            if (value instanceof Reaction) {
                reactions.add(value);
            }
        }
        return reactions;
    }

    /**
     * Returns the set of reactions directly owned by this reactor combined with 
     * the recursive set of all reactions of contained reactors.
     */
    public _getAllReactions(): Set<Reaction> {
        let reactions = this._getReactions();

        // Recursively call this function on child reactors
        // and add their timers to the timers set.
        let children = this._getChildren();
        if(children){
            for(const child of children){
                if(child){
                    let subReactions = child._getAllReactions();
                    for(const subReaction of subReactions){
                        reactions.add(subReaction);
                    }                     
                }
            }
        }
        return reactions;
    }

    /**
     * Iterate through this reactor's attributes,
     * and return the set of its timers.
     */
    public _getTimers(): Set<Timer>{
        // console.log("Getting timers for: " + this)
        let timers = new Set<Timer>();
        for (const [key, value] of Object.entries(this)) {
            if (value instanceof Timer) {
                timers.add(value);
            }
        }
        return timers;
    }

    /**
     * Returns the set of timers directly owned by this reactor combined with 
     * the recursive set of all timers of contained reactors.
     */
    public _getAllTimers(): Set<Timer> {
        // Timers part of this reactor
        let timers = this._getTimers();

        // Recursively call this function on child reactors
        // and add their timers to the timers set.
        let children = this._getChildren();
        if(children){
            for(const child of children){
                if(child){
                    let subTimers = child._getAllTimers();
                    for(const subTimer of subTimers){
                        timers.add(subTimer);
                    }                     
                }
            }
        }
        return timers;
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

    /**
     * Return a string that identifies this component.
     * The name is a path constructed as TopLevelParentName/.../ParentName/ThisReactorsName
     */
    _getFullyQualifiedName(): string {
        
        var path = "";
        if (this._getParent() != null) {
            path = (this._getParent() as Reactor)._getFullyQualifiedName();
        }
        if (path != "") {
            path += "/" + this._getName();
        } else {
            path = this._getName();
        }
        return path;
    }

    _getName():string {
        if (this._myIndex != null && this._myIndex != 0) {
            return this._myName + "(" + this._myIndex + ")";
        } else {
            return this._myName;
        }
    }

    public _setName(name: string) {
        if (this._getParent() != null && (name != this._myName || this._myIndex == null)) {
            //myIndex = parent._getFreshIndex(name); //FIXME: look at former composite
            this._myName = name;
        }
    }

    //A reactor's priority represents its order in the topological sort.
    //The default value of -1 indicates a priority has not been set.
    _priority: number = -1;

    //FIXME: assign in constructor?
    _getPriority(){
        return this._priority;
    }

    _setPriority(priority: number){
        this._priority = priority;
    }

    toString(): string {
        return this._getFullyQualifiedName();
    }

    /**
     * Recursively sets the app attribute for this reactor and all contained reactors to app.
     * @param app The app for this and all contained reactors.
     */
    public _setApp( app: App){
        // console.log("Starting _setApp for: " + this._getFullyQualifiedName());
        console.log("Setting app for: " + this);
        this._app = app;
        // Recursively set the app attribute for all contained reactors to app.
        let children = this._getChildren();
        if(children){
            for(let child of children){
                child._setApp(app);
            }
        }
    }

    /**
     * Recursively traverse all reactors and verify the 
     * parent property of each component correctly matches its location in
     * the reactor hierarchy.
     */
    public _checkAllParents(parent: Reactor | null){
        if(this._parent != parent) throw new Error("The parent property for " + this
            + " does not match the reactor hierarchy.");
        // this._parent = parent;
        let children = this._getChildren();
        for(let child of children){
            child._checkAllParents(this);
        }

        let timers = this._getTimers();
        for(let timer of timers){
            if(timer._getParent() != this) throw new Error("The parent property for " + timer
            + " does not match the reactor hierarchy.");
            // timer._setParent(this);
        }

        // Ports have their parent set in constructor, so verify this was done correctly.
        let ports = this._getPorts();
        for(let port of ports){
            if(port._getParent() != this){
                throw new Error("A port has been incorrectly constructed as an attribute of " +
                                "a different reactor than the parent it was given in its constructor: "
                                + port);
            }
            // port._setParent(this);
        }

        let actions = this._getActions();
        for(let action of actions){
            if(action._getParent() != this) throw new Error("The parent property for " + action
            + " does not match the reactor hierarchy.");
            // action._setParent(this);
        }

    }

    public _hasGrandparent(): boolean {
        if (this._parent != null) {
            return this._parent._hasParent();
        } else {
            return false;
        }
    }

   
    public _hasParent(): boolean {
        if (this._parent != null) {
            return true;
        } else {
            return false;
        }
    }

    public _getGrandparent(): Reactor {
        if(this._hasGrandparent()){
            return ((this._getParent() as Reactor )._getParent() as Reactor);
        } else {
            throw new Error("Cannot get grandparent for a port with no grandparent");
        }
    }

    public _getParent(): Reactor | null {
        return this._parent;
    }

    public _acquire(newParent: Reactor): boolean {
        if (this._parent == null) {
            this._parent = newParent;
            return true;
        } else {
            return false;
        }
    }

    public _release(oldParent: Reactor): boolean {
        if (this._parent == oldParent) {
            this._parent = null;
            this._myIndex = null
            return true;
        } else {
            return false;
        }
    }

    /**
     * Setter method for this reactor's parent.
     * @param parent The reactor containing this one.
     */
    public _setParent(parent: Reactor| null){
        this._parent = parent;
    }
   

    //connect: <T>(source: Port<T>, sink:Port<T>) => void;
    // FIXME: connections mus be done sink to source so that we leverage contravariance of functions!!!
    /**
     * Reactor Constructor.
     * Create a new component; use the constructor name
     * if no name is given.
     * @param {string=} name - Given name
     */
    constructor(parent: Reactor | null, name?:string) {

        this._parent = parent;    
        this._myName = this.constructor.name; // default
        this._myIndex = null;
        // var relations: Map<Port<any>, Set<Port<any>>> = new Map();

        // Set this component's name if specified.
        if (name != null) {
            this._myName = name;
        }


        // Object.assign(this, {
        //     connect<T>(source: Port<T>, sink: Port<T>):void {
        //         // bind T to constrain the type, check connection.
        //         if (source.canConnect(sink)) {
        //             var sinks = relations.get(source); 
        //             if (sinks == null) {
        //                 sinks = new Set();
        //             }
        //             sinks.add(sink);
        //             relations.set(source, sinks);
        //         } else {
        //             throw "Unable to connect."; // FIXME: elaborate error reporting.
        //             //throw "Cannot connect " + source.getFullyQualifiedName() + " to " + sink.getFullyQualifiedName() + ".";
        //         }
        //     // FIXME: check the graph for cycles, etc.
            
        //     }
        // });

        // Add it to a container if one is specified.
        // Note: the call to _add will invoke this._acquire,
        // so this code must be executed _after_ assigning
        // the _acquire function in the constructor.
        // if (parent != null) {
        //     //parent._add(this); // FIXME: add container capability to Reactor
        // }
    }
}

export abstract class Port<T> implements Named {
    
    // The reactor containing this port
    // This attribute is set by the parent reactor's constructor.
    protected parent: Reactor;

    protected _value: TimestampedValue<T> | null = null;
    
    /**
     * Getter function for a ports _value attribute.
     * WARNING: Do not call this function in a reaction.
     * Use get() instead. This function should only be used
     * by the runtime to implement user-facing functions such
     * as get().
     */
    public _getValue(){
        return this._value;
    }

    /***** Priviledged functions *****/

    public abstract connect(source: Port<T>): void;
    public abstract canConnect(source: Port<T>): boolean;
    public abstract get():T | null;
    // public abstract set(value: T): void;

    _connectedSinkPorts: Set<Port<T>> = new Set<Port<T>>();
    _connectedSourcePort: Port<T>| null = null;


    public _getFullyQualifiedName(): string {
        return this.parent._getFullyQualifiedName() 
            + "/" + this._getName();
    }

    /**
     * Setter method for a port's parent attribute.
     * @param parent The reactor this port is attached to.
     */
    public _setParent(parent: Reactor){
        this.parent = parent;
    }

    /**
     * Getter method for a port's parent attribute.
     */
    public _getParent(){
        return this.parent;
    }

    public _hasParent(): boolean {
        if(this.parent){
            return true;
        } else {
            return false;
        }
    }

    public _hasGrandparent():boolean {
        if(this.parent && this.parent._hasParent()){
            return true;
        } else {
            return false;
        }
    }

    /* Return a globally unique identifier. */
    public _getName(): string {
        var alt = "";
        for (const [key, value] of Object.entries(this.parent)) {
            if (value === this) { // do hasOwnProperty check too?
                return `${key}`;
            }
        }
        return "anonymous";
    }

    /**
     * //FIXME: We will probably have to change something
     * here when we implement mutations.
     * Assigns a value to this port at the current logical time.
     * Put the reactions this port triggers on the reaction 
     * queue and recursively invoke this function on all connected output ports.
     * Note: It is considered incorrect for a reaction to directly call this
     * function on a port. Instead, reactions should call the "set()" function on 
     * an OutPort. InPorts should not be set().
     * @param value The value to assign to this output port.
     */
    _writeValue(value: T):void {
        // console.log("calling _writeValue on: " + this);
        if(this instanceof InPort){
            // Input ports can trigger reactions for the reactor
            // they are attached to.
            for (let r of this.parent._reactions) {
                if (r.triggers.includes(this)) {
                    //Create a PrioritySetNode for this reaction and push the node to the reaction queue
                    let prioritizedReaction = new PrioritizedReaction(r, this.parent._app._getReactionID());
                    this.parent._app._scheduleReaction(prioritizedReaction);
                }
            }
        } else {
            // Output ports can trigger reactions for a reactor containing the
            // reactor they are attached to.
            this._value = [this.parent._app._getcurrentlogicaltime(), value];
            if(this._hasGrandparent() && this.parent._getParent()){
                for (let r of (this.parent._getParent() as Reactor)._reactions) {
                    if (r.triggers.includes(this)) {
                        //Create a PrioritySetNode for this reaction and push the node to the reaction queue
                        let prioritizedReaction = new PrioritizedReaction(r, this.parent._app._getReactionID());
                        this.parent._app._scheduleReaction(prioritizedReaction);
                    }
                }
            }
        }

        for(const port of this._connectedSinkPorts){
            port._writeValue(value);
        }
    }

    /* Construct a new port. */
    /**
     * Port constructor.
     * @param parent 
     */
    constructor(parent: Reactor) {
        this.parent = parent;
    }

    toString(): string {
        return this._getFullyQualifiedName();
    }
}


export class OutPort<T> extends Port<T> implements Port<T>, Writable<T> {

    // value: TimestampedValue<T> | null = null;
    // _connectedSinkPorts: Set<Port<T>> = new Set<Port<T>>();
    // _connectedSourcePort: Port<T> | null = null;

    /***** Priviledged functions *****/

    public isPresent(){
        if(this._value && timeInstantsAreEqual(this._value[0],this.parent._app._getcurrentlogicaltime() )){
            return true;
        } else {
            return false;
        }
    }

    public get(): T | null {
        if(this._value && this.isPresent()){
            return this._value[1];
        } else {
            return null;
        }
    }

    /**
     * Return the set of all InPorts connected to this OutPort
     * directly or indirectly as a sink.
     */
    public getAllConnectedSinkInPorts(){
        let sinkInPorts = new Set<InPort<any>>();
        for( let connected of this._connectedSinkPorts){
            if(connected instanceof InPort){
                sinkInPorts.add(connected);
            } else {
                let recursiveSinkInPorts = (connected as OutPort<any>).getAllConnectedSinkInPorts();
                for (let newSink of recursiveSinkInPorts){
                    sinkInPorts.add(newSink);
                }
            }
        }
        return sinkInPorts;
    }

    /**
     * Returns true if this port can be connected to sink. False otherwise. 
     * @param sink The port to test connection against. 
     */
    public canConnect(sink: Port<T>): boolean {

        // Self-loops are not permitted.
        if(this === sink){
            return false;
        }

        // OUT to In
        // Reactor with input port must be at the same level of hierarchy as
        // reactor with output port.
        if(sink instanceof InPort){ 
            if(this._getParent()._getParent() == sink._getParent()._getParent()){
                return true;
            } else {
                return false;
            }
        
        // OUT to OUT
        // This reactor must be the child of sink's reactor 
        } else {
            if(this._getParent()._getParent() == sink._getParent()){
                return true;
            } else {
                return false;
            }
        }
    }

    /**
    * Write a value to this OutPort and recursively transmit the value to connected
    * ports while triggering reactions triggered by that port. 
    * @param value The value to be written to this port.
    */
    public set(value: T):void {
        this._writeValue(value);
    }

    // NOTE: Due to assymmetry (subtyping) we cannot allow connecting 
    // sinks to sources. It must always be source to sink. Disconnect 
    // does not have this problem.

    /**
    * Connect this OutPort to a downstream port.
    * @param sink The port to which this OutPort should be connected.
    */
    connect(sink: Port<T>):void {
        console.log("connecting " + this + " and " + sink);
        this._connectedSinkPorts.add(sink);
        sink._connectedSourcePort = this;
    
        // var container = parent._getContainer();
        // if (container != null) {
        //     container.connect(this, sink);
        // } else {
        //     throw "Unable to connect: add the port's component to a container first.";
        // }
    }

    public disconnect(sink: Port<T>): void {
        // this.connectedPort = null;
        this._connectedSinkPorts.delete(sink);
        sink._connectedSourcePort = null;
    }
    
    // disconnect(direction:"upstream"|"downstream"|"both"="both"): void {
    //     var component = parent;
    //     var container = component._getContainer();

    //     if (direction == "upstream" || direction == "both") {
    //         if (component instanceof Reactor) {    
    //             // OUT to OUT
    //             //component._disconnectContainedReceivers(this); //FIXME: add a transfer reaction
    //         }
    //     }

    //     if (direction == "downstream" || direction == "both") {
    //         // OUT to IN
    //         // OUT to OUT
    //         if (container != null) {
    //             //container._disconnectContainedSource(this);    //FIXME: add a transfer reaction
    //         }
    //     }
    // }

    toString(): string {
        return this._getFullyQualifiedName();
    }

}

export class InPort<T> extends Port<T> implements Readable<T> {

    /**
     * If an InPort has a null value for its connectedPort it is disconnected.
     * A non-null connectedPort is connected to the specified port.
     */
    _connectedSinkPorts: Set<Port<T>> = new Set<Port<T>>();
    _connectedSourcePort: Port<T> | null = null;
    // _name: string = "";
    //_receivers: Set<Port<T>>;
    //_parent: Component; // $ReadOnly ?
    //_persist: boolean;

    /***** Priviledged functions *****/      
    //send: (value: ?$Subtype<T>, delay?:number) => void;
    //writeValue: (value: T ) => void;

    /**
     * Returns true if the connected port is directly or indirectly connected to
     * an output port with a value set at the current logical time. Returns false otherwise
     * Throws an error if not connected directly or indirectly to an output port.
     */
    public isPresent():boolean {
        if(this._connectedSourcePort){
            if(this._connectedSourcePort instanceof OutPort){
                let portValue = this._connectedSourcePort._getValue();
                if( portValue === null ||
                    ! timeInstantsAreEqual(portValue[0], this.parent._app._getcurrentlogicaltime() )){
                        return false;
                    } else {
                        return true;
                    }
            } else if(this._connectedSourcePort instanceof InPort) {
                return this._connectedSourcePort.isPresent();
            } else {
                throw new Error("Can only call isPresent() on an InPort or an OutPort");
            }
        } else {
            throw new Error("Cannot test a disconnected input port for a present value.")
        }
    }

    /**
     * Obtains a value from the connected port. If connected to an output port, this
     * can be done directly. If connected to an input port, recursively call get on that.
     * Throws an error if this port is not connected to anything
     * or is connected to a chain of input ports which is not terminated by a connection
     * to an output port.
     * Will return null if the connected output did not have its value set at the current
     * logical time.
     */
    get():T | null {
        // console.log("calling get on " + this);
        if(this._connectedSourcePort){
            if(this._connectedSourcePort instanceof OutPort){
                let portValue = this._connectedSourcePort._getValue();
                if(portValue && this.isPresent()){
                    return portValue[1];
                } else {
                    return null;
                }
            } else {
                return this._connectedSourcePort.get();
            }
        } else {
            throw new Error("Cannot get value from a disconnected port.")
        }
    }

    /**
     * Returns true if this port can be connected to source. False otherwise. 
     * @param sink The port to test connection against. 
     */
    canConnect(sink: Port<T>): boolean {
        
        //Self loops are not allowed.
        if(sink == this){
            return false;
        }
        if(sink instanceof InPort){
            // IN to IN
            // sink's reactor must be the child of this one.
            if(sink.parent._getParent() == this.parent){
                return true;
            } else {
                return false;
            }
        } else{
            // IN to OUT
            // An output port can't be the sink of an input port.
            return false;

    //     var thisComponent = parent;
    //     var thisContainer = parent._getContainer();

    //     if (thisComponent instanceof Reactor 
    //         && sink instanceof InPort 
    //         && sink.hasGrandparent(thisComponent)) {
    //         return true;
    //     } else {
    //         return false;
    //     }
        }
    }

    /**
     * Connect this InPort to a downstream port.
     * @param sink the port to connect to.
     */
    public connect(sink: Port<T>):void {
        console.log("connecting " + this + " and " + sink);
        this._connectedSinkPorts.add(sink)
        sink._connectedSourcePort = this;
    }

    public disconnect(sink: Port<T>): void {
        this._connectedSinkPorts.delete(this);
        sink._connectedSourcePort = null;
    }
        // disconnect(direction:"upstream"|"downstream"|"both"="both"): void {
        //     var component = parent;
        //     var container = component._getContainer();

        //     if (direction == "upstream" || direction == "both") {
        //         if (container != null) {
        //             // OUT to IN
        //             // IN to IN
        //             //container._disconnectContainedReceivers(this); // FIXME: this should result in the removal of a transfer reactions
        //         }    
        //     }

        //     if (direction == "downstream" || direction == "both") {
        //         if (component instanceof Reactor) {
        //             // IN to IN
        //             //component._disconnectContainedSource(this);
        //         }
        //         if (container != null) {
        //             // IN to OUT
        //             //container._disconnectContainedSource(this);
        //         }
        //     }
        // }

    toString(): string {
        return this._getFullyQualifiedName();
    }
}


/**
 * This class matches a Trigger to the Reactions it triggers.
 * When an event caused by a Trigger comes off the event queue, its
 * matching reactions should be put on the the reaction queue 
 */
export class TriggerMap{
    _tMap: Map<Trigger, Set<Reaction>> = new Map<Trigger, Set<Reaction>>();

    /**
     * Establish the mapping for a Reaction.
     */
    registerReaction(r: Reaction){
        for(let trigger of r.triggers){
            let reactionSet = this._tMap.get(trigger);
            if(reactionSet){
                if(! reactionSet.has(r)){
                    reactionSet.add(r);
                    this._tMap.set(trigger, reactionSet);
                }
                //If this reaction is already mapped to the trigger,
                //do nothing.
            } else {
                //This is the first reaction mapped to this trigger,
                //so create a new reaction set for it.
                reactionSet = new Set<Reaction>();
                reactionSet.add(r);
                this._tMap.set(trigger, reactionSet);
            }
        }
    }

    /**
     * Get the set of reactions for a trigger.
     */
    getReactions(t: Trigger){
        return this._tMap.get(t);
    }

    /**
     * FIXME
     */
    deregisterReaction(e: Event<any>){
        //FIXME
    }
}

//FIXME: Move runtime from globals into here.
export class App extends Reactor{
    
    // // FIXME: add some logging facility here
    // name: string;

    /**
     * If not null, finish execution with success, this time interval after
     * the start of execution.
     */
    private _executionTimeout:TimeInterval | null = null;

    /**
     * The numeric time at which execution should be stopped.
     * Determined from _executionTimeout and startingWallTime.
     */
    private _relativeExecutionTimeout: NumericTimeInterval;

    /**
     * Prioritized queues used to manage the execution of reactions and events.
     */
    private _reactionQ = new PrioritySet<number,number>();
    private _eventQ = new PrioritySet<number,TimeInstant>();

    /**
     * The current time, made available so actions may be scheduled relative to it.
     */
    private _currentLogicalTime: TimeInstant;

    /**
     * The physical time when execution began expressed as [seconds, nanoseconds]
     * elapsed since January 1, 1970 00:00:00 UTC.
     * Initialized in start()
     */
    private _startingWallTime: NumericTimeInterval;

    /**
     * Used to look out for the same action scheduled twice at a logical time. 
     */ 
    private _observedActionEvents : Map<Action<any>, Event<any>> = new Map<Action<any>, Event<any>>();

    // FIXME: Use BigInt instead of number?
    /** 
     * Track IDs assigned to reactions and events
     */
    private _reactionIDCount = 0;
    private _eventIDCount = 0;    

    /**
     * Acquire all the app's timers and call setup on each one.
     */
    private _startTimers = function(){
        let timers: Set<Timer> = this._getAllTimers();
        for(let t of timers){
            console.log("setting up timer " + t);
            t.setup();
        }
    };

    /**
     * Register all the app's reactions with their triggers.
     */
    private _registerReactions = function(){
        let reactions: Set<Reaction> = this._getAllReactions();
        // console.log("reactions set in _registerReactions is: " + reactions);
        for(let r of reactions){
            // console.log("registering: " + r);
            r.register();
        }
    }


    /**
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
     * allowed to continue indefinately.
     * Otherwise, call failureCallback when there are no events in the queue.
     * 
     * FIXME: Implement a keepalive option so execution may continue if
     * there are no more events in the queue.
     * @param successCallback Callback to be invoked when execution has terminated
     * in an expected way.
     * @param failureCallback Callback to be invoked when execution has terminated
     * in an unexpected way.
     */
    private _next(successCallback: ()=> void, failureCallback: () => void){
        // console.log("starting _next");
        let currentHead = this._eventQ.peek();
        while(currentHead){
            let currentPhysicalTime:NumericTimeInterval = microtimeToNumeric(microtime.now());
            // console.log("current physical time in next is: " + currentPhysicalTime);
            
            //If execution has gone on for longer than the execution timeout,
            //terminate execution with success.
            if(this._executionTimeout){
                //const timeoutInterval: TimeInterval= timeIntervalToNumeric(_executionTimeout);
                if(compareNumericTimeIntervals( this._relativeExecutionTimeout, currentPhysicalTime)){
                    console.log("Execution timeout reached. Terminating runtime with success.");
                    successCallback();
                    return;
                }
            }
            if(compareNumericTimeIntervals(currentPhysicalTime, currentHead._priority[0] )){
                //Physical time is behind logical time.
                let physicalTimeGap = numericTimeDifference(currentHead._priority[0], currentPhysicalTime, );
            
                //Wait until min of (execution timeout and the next event) and try again.
                let timeout:NumericTimeInterval;
                if(this._executionTimeout && compareNumericTimeIntervals(this._relativeExecutionTimeout, physicalTimeGap)){
                    timeout = this._relativeExecutionTimeout;
                } else {
                    timeout = physicalTimeGap;
                }
                console.log("Runtime set a timeout at physical time: " + currentPhysicalTime +
                 " for an event with logical time: " + currentHead._priority[0]);
                // console.log("Runtime set a timeout with physicalTimeGap: " + physicalTimeGap);
                // console.log("currentPhysicalTime: " + currentPhysicalTime);
                // console.log("next logical time: " + currentHead._priority[0]);
                // console.log("physicalTimeGap was " + physicalTimeGap);

                //Nanotimer https://www.npmjs.com/package/nanotimer accepts timeout
                //specified by a string followed by a letter indicating the units.
                //Use n for nanoseconds. We will have to 0 pad timeout[1] if it's
                //string representation isn't 9 digits long.
                let nTimer = new NanoTimer();
                let nanoSecString = timeout[1].toString();

                //FIXME: this test will be unecessary later on when we're more
                //confident everything is working correctly.
                if( nanoSecString.length > 9){
                    throw new Error("Tried to set a timeout for an invalid NumericTimeInterval with nanoseconds: " +
                        nanoSecString );
                }

                //Convert the timeout to a nanotimer compatible string.
                let padding = "";
                for(let i = 0; i < 9 - nanoSecString.length; i++){
                    padding = "0" + padding 
                }
                let timeoutString = timeout[0].toString() + padding + nanoSecString + "n"; 
                nTimer.setTimeout(this._next.bind(this), [successCallback, failureCallback], timeoutString);
                
                return;
            } else {
                //Physical time has caught up, so advance logical time
                this._currentLogicalTime = currentHead._priority;
                console.log("At least one event is ready to be processed at logical time: "
                 + currentHead._priority + " and physical time: " + currentPhysicalTime );
                // console.log("currentPhysicalTime: " + currentPhysicalTime);
                //console.log("physicalTimeGap was " + physicalTimeGap);

                // Using a Set data structure ensures a reaction triggered by
                // multiple events at the same logical time will only react once.
                let triggersNow = new Set<Reaction>();

                // Keep track of actions at this logical time.
                // If the same action has been scheduled twice
                // make sure it gets the correct (last assigned) value.
                this._observedActionEvents.clear();


                // Remove all simultaneous events from the queue.
                // Reschedule timers, assign action values, and put the triggered reactions on
                // the reaction queue.
                // This loop should always execute at least once.
                while(currentHead && timeInstantsAreEqual(currentHead._priority, this._currentLogicalTime)){

                    //An explicit type assertion is needed because we know the
                    //eventQ contains PrioritizedEvents, but the compiler doesn't know that.
                    let trigger: Trigger = (currentHead as Event<any>).cause;
                    
                    if(trigger instanceof Timer){
                        trigger.reschedule();
                    }

                    if(trigger instanceof Action){
                        // Check if this action has been seen before at this logical time.
                        if(this._observedActionEvents.has(trigger) ){
                            // Whichever event for this action has a greater eventID
                            // occurred later and it determines the value. 
                            if( currentHead._id > (this._observedActionEvents.get(trigger) as Event<any>)._id ){
                                trigger.value = (currentHead as Event<any>).value;
                                trigger.timestamp = this._currentLogicalTime;
                            }
                        } else {
                            // Action has not been seen before.
                            this._observedActionEvents.set(trigger, (currentHead as Event<any>));
                            trigger.value = (currentHead as Event<any>).value;
                            trigger.timestamp =  this._currentLogicalTime;
                        }
                    }
                    // console.log("Before triggermap in next");
                    let toTrigger = this._triggerMap.getReactions(trigger);
                    // console.log(toTrigger);
                    // console.log("after triggermap in next");
                    if(toTrigger){
                        for(let reaction of toTrigger){

                            //Push this reaction to the queue when we are done
                            //processing events.
                            triggersNow.add(reaction);
                        }
                    }
                    this._eventQ.pop();
                    currentHead = this._eventQ.peek();
                }
                
                for (let reaction of triggersNow){
                    // console.log("Pushing new reaction onto queue");
                    // console.log(reaction);
                    let prioritizedReaction = new PrioritizedReaction(reaction, this._getReactionID());
                    this._reactionQ.push(prioritizedReaction);
                }
                
                let headReaction = this._reactionQ.pop();
                while(headReaction){
                    // Explicit type annotation because reactionQ contains PrioritizedReactions.
                    let r = (headReaction as PrioritizedReaction).r
                    
                    // Test if this reaction has a deadline which has been violated.
                    // This is the case if the reaction has a registered deadline and
                    // logical time + timeout < physical time
                    if(r.deadline && compareNumericTimeIntervals( 
                            numericTimeSum(this._currentLogicalTime[0], timeIntervalToNumeric(r.deadline.getTimeout())),
                            currentPhysicalTime)){
                        console.log("handling deadline violation");
                        r.deadline.handler();
                    } else {
                        console.log("reacting...");
                        r.react();
                    }
                    headReaction = this._reactionQ.pop();
                }

                //A new Action event may have been pushed onto the event queue by one of
                //the reactions at this logical time.
                currentHead = this._eventQ.peek();
            }

            //The next iteration of the outer loop is ready because
            //currentHead is either null, or a future event
        }
        //Falling out of the while loop means the eventQ is empty.
        console.log("Terminating runtime with failure due to empty event queue.");
        failureCallback();
        return;
        //FIXME: keep going if the keepalive command-line option has been given
    }

    /**
     * FIXME
     * @param executionTimeout 
     * @param name 
     */
    constructor( executionTimeout: TimeInterval | null, name?: string) {
        super(null, name)
        
        // Note: this.parent is initialized to null because an app is a top
        // level reactor.
        this._parent = null;

        this._executionTimeout = executionTimeout;
    }

    /**
     * Maps triggers coming off the event queue to the reactions they trigger.
     * Public because reactions need to register themselves with this structure
     * when they're created. 
     */
    public _triggerMap: TriggerMap = new TriggerMap();

    /**
     * Public method to push events on the event queue. 
     * @param e Prioritized event to push onto the event queue.
     */
    public _scheduleEvent(e: Event<any>){  
        this._eventQ.push(e);
    }

    /**
     * Public method to push reaction on the reaction queue. 
     * @param e Prioritized reaction to push onto the reaction queue.
     */
    public _scheduleReaction(r: PrioritizedReaction){  
        this._reactionQ.push(r);
    }

    /**
     * Obtain a unique identifier for the reaction.
     */
    public _getReactionID(){
        return this._reactionIDCount++;
    }

    /**
     * Public getter for logical time. 
     */
    public _getcurrentlogicaltime(){
        return this._currentLogicalTime;
    }
    
    /**
     * Obtain a unique identifier for the event.
     * Note: The monotonicly increasing nature of eventIDs
     * is used to resolve priority between duplicate events with the same
     * timestamp in the eventQ.
     */
    public _geteventID(){
        return this._eventIDCount++;
    }

    /**
     * Public getter for starting wall time.
     */
    public _getStartingWallTime(){
        return this._startingWallTime
    }

    /**
     * Assign a priority to each reaction in the app.
     * A lower priority signifies precedence for one reaction
     * over another. 
     */
    public _setReactionPriorities(){
        let unsetReactions = this._getAllReactions();
        let setReactions = new Set<Reaction>();
        let unsetOutPorts = new Set<OutPort<any>>();

        // InPorts connected to set OutPorts
        let setInPorts = new Set<InPort<any>>();

        // A map relating OutPorts to the reactions
        // which must be set first. 
        let outPortDependsOn = new Map<OutPort<any>, Set<Reaction>>();

        // A map relating reactions to the InPorts
        // or reactions which must first be set.
        let reactionDependsOn = new Map<Reaction, Set<Reaction | InPort<any>>>();

        // Initialize outPortDependsOn and unsetOutPorts
        for(let r of unsetReactions){
            for(let e of r.getEffects()){
                if(e instanceof OutPort){
                    unsetOutPorts.add(e);
                    if(outPortDependsOn.has(e)){
                        (outPortDependsOn.get(e) as Set<Reaction>).add(r);
                    } else {
                        let newReactionSet = new Set<Reaction>();
                        newReactionSet.add(r);
                        outPortDependsOn.set(e, newReactionSet);
                    }
                }
            }
        }

        // Initialize reactionDependsOn
        for(let r of unsetReactions){
            reactionDependsOn.set(r, new Set<Reaction| InPort<any>>());
            // Add InPorts from uses
            for(let u of r.getUses()){
                (reactionDependsOn.get(r) as Set<Reaction| InPort<any>> ).add(u);
            }
            let parentReactions = r.state._reactions;
            if(! parentReactions.includes(r)){
                throw new Error(" Reaction " + r + "is not included in its parent's "
                + " array of reactions");
            }
            // Add preceding reactions from parent's reactions array
            for (let i = 0; parentReactions[i] != r ; i++ ){
                (reactionDependsOn.get(r) as Set<Reaction| InPort<any>> ).add(parentReactions[i]);
            }
        }

        let priorityCount = 0;
        while(unsetReactions.size > 0){
            // Find a reaction in unsetReactions with no unset dependencies.
            // Assign it the next lowest priority, and remove
            // it from unsetReactions. Throw an error, identifying a cycle if 
            // this process stops before all reactions are set.
            let newlySetReactions = new Set<Reaction>();
            for( let r of unsetReactions ){
                let ready = true;
                for( let depend of (reactionDependsOn.get(r) as Set<Reaction | InPort<any>>)){
                    if(depend instanceof Reaction){
                        if (unsetReactions.has(depend)){
                            ready = false;
                            break;
                        }
                    } else {
                        if (! setInPorts.has(depend)){
                            ready = false;
                            break;
                        }
                    }
                }
                if(ready){
                    console.log("Setting priority for reaction " + r + " to " + priorityCount);
                    // This reaction has no dependencies. Set its priority.
                    r.setPriority(priorityCount++);
                    newlySetReactions.add(r);
                }
            }

            // If no new reactions with met dependencies are
            // found on this iteration while unset reactions remain,
            // there must be a cycle.
            if(newlySetReactions.size == 0){
                throw new Error("Cycle detected in reaction precedence graph.");
            }

            // Move newlySetReactions from unsetReactions
            // to setReactions.
            for(let toSet of newlySetReactions){
                unsetReactions.delete(toSet);
                setReactions.add(toSet)
            }

            // See if any OutPorts are ready to be set
            for(let o of unsetOutPorts){
                let ready = false;
                for(let portReaction of (outPortDependsOn.get(o) as Set<Reaction>)){
                    if(unsetReactions.has(portReaction)){
                        ready = false;
                        break;
                    }
                }
                if(ready){
                    // Remove the OutPort from unsetOutPorts and set all connected
                    // InPorts
                    unsetOutPorts.delete(o);
                    let connectedInPorts = o.getAllConnectedSinkInPorts();
                    for(let connectedInPort of connectedInPorts){
                        setInPorts.add(connectedInPort);
                    }
                }
            }
        }
    }

    public _start(successCallback: () => void , failureCallback: () => void):void {
        // Recursively check the parent attribute for this and all contained reactors and
        // and components, i.e. ports, actions, and timers have been set correctly.
        this._checkAllParents(null);
        // Recursively set the app attribute for this and all contained reactors to this.
        this._setApp(this);
        // Set reactions using a topological sort of the dependency graph.
        this._setReactionPriorities();
        // Recursively register reactions of contained reactors with triggers in the triggerMap.
        this._registerReactions();
        // console.log(this.triggerMap);
        this._startingWallTime = microtimeToNumeric(microtime.now());
        this._currentLogicalTime = [ this._startingWallTime, 0];
        if(this._executionTimeout !== null){
            this._relativeExecutionTimeout = numericTimeSum(this._startingWallTime, timeIntervalToNumeric(this._executionTimeout));
        }
        this._startTimers();
        this._next(successCallback, failureCallback);

    }



    //FIXME:
    // stop():void {

    // }

    //FIXME: What is this function supposed to do?
    // _checkTypes() {

    // }
}

//---------------------------------------------------------------------//
// Commented Out Code                                                 //
//---------------------------------------------------------------------//
//For whatever reason, code I don't want to delete just yet.


/**
 * An event is caused by a timer, caused by an input, or caused by an internal event
 * It occurs at a particular time instant and carries an arbitrary data type as value.
 * There are three kinds of events: Timer, Input, and Internal.
 * They all have the same properties.
 */
//In the C implementation an event has a time, trigger, and value.

//FIXME: Rename this class because it conflicts with a built in
//class in typescript. Maybe make it an interface or abstract class?
// export class Event {
//     time: TimeInstant;
//     cause: Trigger;
//     value: any;

//     //FIXME: make value optional
//     constructor(cause: Trigger, time: TimeInstant, value: any){
//         this.time = time;
//         this.cause = cause;
//         this.value = value;
//     }
// }



//Moved to commented section because this interface is redundant with new
//Port base class. I combined base class with these functions 
//WARNING: Out of date documentation.
/**
 * An interface for ports. Each port is associated with a parent component.
 * Ports may be connected to downstream ports using connect(). 
 * Connections between ports can be destroyed using disconnect().
 * Messages can be sent via a port using send(). Message delivery is immediate
 * unless a delay is specified.
 */
// export interface Port<T> extends  Named {

//     hasGrandparent: (container:Reactor) => boolean;
//     hasParent: (component: Reactor) => boolean;

//     connect: (source: Port<T>) => void;
//     canConnect(source: Port<T>): boolean;


// }


//FIXME: I don't know what the purpose of this is.
/**
 * A generic container for components.
 */
// export interface Container<T: Reactor> {

//     /**
//      * Add a list of elements to this container. Duplicate entries will
//      * not be kept.
//      * @param {T} element
//      */
//     _add(...elements: Array<T>): void;

//     /**
//      * Return whether or not the argument is present in the container.
//      * @param {T} element
//      */
//     _contains(element: T): boolean;

//     /**
//      * Remove an element from this container.
//      * @param {string} name
//      */
//     _remove(element: T): void;

//     //_move(element: T, destination: Container<T: Component>)

// }


//An interface for classes implementing a react function.
//Both reactions and timers react to events on the event queue
// export interface Reaction {
//     react:() => void;
//     triggers: Array<Trigger>;
//     priority: number;
// }


//Matt: An action is an internal event so I think there's a lot of overlap with
//that class and the below code.
//BEGIN COMMENTED OUT ACTION CLASS

/**
 * An action denotes a self-scheduled event. If an action is instantiated
 * without a delay, then the time interval between the moment of scheduling
 * this action (cause), and a resulting reaction (effect) will be determined
 * upon the call to schedule. If a delay _is_ specified, it is considered
 * constant and cannot be overridden using the delay argument in a call to 
 * schedule().
 */
// export class Action<T> implements Trigger {
//     get: () => T | null;

//     /**
//      * Schedule this action. If additionalDelay is 0 or unspecified, the action 
//      * will occur at the current logical time plus one micro step.
//      */
//     schedule: (additionalDelay?:TimeInterval, value?:T) => TimeInstant;

//     constructor(parent:Reactor, delay?:TimeInterval) { 
//         var _value: T;

//         Object.assign({
//             get(): T | null {
//                 return _value;
//             }
//             // FIXME: add writeValue
//         });

//         Object.assign(this, {
//             schedule(additionalDelay:TimeInterval, value?:T): TimeInstant {
//                 let numericDelay: number;
                
//                 //FIXME
//                 // if(additionalDelay == null || additionalDelay == 0
//                 //      || additionalDelay[0] == 0){


//                 // }

//                 // if ( (delay == null || delay === 0) &&  ) {
//                 //     numericDelay = timeIntervalToNumber(additionalDelay);
//                 // } else {
//                 //     if (additionalDelay != null && additionalDelay !== 0) {
//                 //         delay[0] += timeIntervalToNumber(additionalDelay);
//                 //     }
//                 // }
//                 // return _schedule(this, delay, value);
//             }
//         });
//     }

//     unschedule(handle: TimeInstant):void {
//         // FIXME
//     }
// }

//END COMMENTED OUT ACTION CLASS

// export interface Schedulable<T> {
//     schedule: (additionalDelay?:TimeInterval, value?:T) => TimeInstant;
//     unschedule(handle: TimeInstant):void;
// }

//FIXME: I believe the current LF spec has all inputs contained.
// class ContainedInput<T> implements Writable<T> {
    
//     set: (value: T | null) => void;

//     constructor(reactor:Reactor, port:InPort<T>) {
//         var valid = true;
//         if (!port.hasParent(reactor)) {
//             console.log("WARNING: port " + port._getFullyQualifiedName()
//                 + "is improperly used as a contained port; "
//                 + "set() will have no effect.");
//             valid = false;
//         }

//         Object.assign(this, {
//             set(value:T | null): void {
//                 if (valid) {
//                     return port.writeValue(reactor, value);
//                 }
//             }
//         });
//     }
// }

//FIXME: I believe the current LF spec has all outputs contained.
// class ContainedOutput<T> implements Readable<T> {
//     get: () => T | null; // FIXME: remove readable from output!!
    
//     constructor(reactor:Reactor, port:OutPort<T>) {
//         var valid = true;
//         if (!port.hasParent(reactor)) {
//             console.log("WARNING: port " + port._getFullyQualifiedName()
//                 + "is improperly used as a contained port; "
//                 + "get() will always return null.");
//             valid = false;
//         }

//         Object.assign(this, {
//             get(): T | null {
//                 if (!valid) {
//                     return null;
//                 } else {
//                     return port.get();
//                 }
//             }
//         });
//     }
// }

// class CallerPort<A,R> implements Connectable<CalleePort<A,R>> {

//     constructor() {

//     }

//     call() {

//     }

//     connect(sink: CalleePort<A,R>):void {
//         return;
//     }

//     canConnect(sink: CalleePort<A,R>):boolean {
//         return true;
//     }

//     invokeRPC: (arguments: A, delay?:number) => R;

// }

// class CalleePort<A,R> {

// }

// export interface Connectable<T> {
//     +connect: (source: T) => void;
//     +canConnect: (source: T) => boolean;
// }


// NOTE: composite IDLE or REACTING.
// If IDLE, get real time, of REACTING use T+1

// export class Composite extends Component implements Container<Component>, Reactor {

//     _getFreshIndex: (string) => number;
//     _disconnectContainedReceivers: (port: Port<*>) => void;
//     _disconnectContainedSource: (port: Port<*>) => void;
//     _getGraph: () => string;

//     connect: <T>(source: Port<T>, sink:Port<T>) => void;
//     //disconnect: (port: Port<*>, direction?:"upstream"|"downstream"|"both") => void;
//     schedule: <T>(action:Action<T>, value:any, repeat?:boolean) => number;
//     getCurrentTime: () => Time;

//     _init:() => void;
//     _wrapup: () => void;
//     _react:() => void;  
//     _reactions:$ReadOnlyArray<[Array<Trigger<*>>, Reaction<any, any>]>;

//     constructor(parent:?Composite, name?:string) {
//         super(parent, name);

//         /* Private variables */
//         var relations: Map<Port<any>, Set<Port<any>>> = new Map();
        
//         //var eventQ: Map<Time, Map<*>, *> = new Map();

//         // queue for delayed triggers
//         var triggerQ: Map<number, [Map<Action<any>, any>]> = new Map();

//         // queue for delayed sends
//         var sendQ: Map<number, [Map<Port<any>, any>]> = new Map();

//         var indices: Map<string, number> = new Map();

//         var actors: Set<ReActor> = new Set();

//         // we need to express dependencies between reactions, not between ports
//         var dependencies: Map<Reaction<mixed>, Reaction<mixed>> = new Map();

//         Object.assign(this, {
//             _init() {
//                 for (let a of actors) {
//                     for (let r of a._reactions) {

//                     }
//                 }
//             }
//         });
        

//         Object.assign(this, {
//             schedule<T>(action:Action<T>, value:any, repeat?:boolean): number {
                
//                 return 0;
//             }
//         });
//         // We don't want to run ahead of realtime, because actors can produce spontaneous events that need to be stamped with 
//         // wallclock time, and we don't want these timestamps to be "in the past".
//         // DAC Properties A1-9.
//         // Simple examples. Which should those be?
//         // First one to start with: sensor computation actuator
//         // Introduce notion of a deadline
//         // Why on the local platform, model should not get ahead.
//         // Example 1: Synchronization to real time and deadlines
//         // Example 2: Why delay has to wait
//         // Example 3: shut off the lights some time after the switch has been flipped.
//         // Reason to have the deadline definition as stated: detectability. Suppose the start deadline cannot be met; the
//         // reaction should not be carried out (and then the violation be reported on).
        

//         Object.assign(this, {
//             // FIXME: We may want to wrap this into something like a change request and 
//             // let the composite handle it at the next microstep.
//             connect<T>(source: Port<T>, sink: Port<T>):void {
//                 // bind T to constrain the type, check connection.
//                 if (source.canConnect(sink)) {
//                     var sinks = relations.get(source); 
//                     if (sinks == null) {
//                         sinks = new Set();
//                     }
//                     sinks.add(sink);
//                     relations.set(source, sinks);
//                 } else {
//                     throw "Unable to connect."; // FIXME: elaborate error reporting.
//                     //throw "Cannot connect " + source.getFullyQualifiedName() + " to " + sink.getFullyQualifiedName() + ".";
//                 }
//             // FIXME: check the graph for cycles, etc.
            
//             }
//         });
//         // FIXME: persistent <=> default
//         // Comments from Stoyke. 1) What if you want non-determinism? Parameter store. Stores the parameters that you are learning.
//         // Fairly common strategy. Parallel processes. All updating the parm store asynchronously.
//         // 2) How to handle dynamic instantiation?

//         Object.assign(this, {
//             _getFreshIndex(name: string): number {
//                 var index = 0;
//                 if (indices.has(name)) {
//                     index = indices.get(name)+1;
//                     indices.set(name, index);
//                 } else {
//                     indices.set(name, index);
//                 }
//                 return index;
//             }
//         });

//         Object.assign(this, {
//             _react() {
//                 for (var prop in this) {
//                     if (prop instanceof InPort) {
//                         console.log("port: " + prop.toString());
//                     }

//                     if (prop instanceof OutPort) {
//                         console.log("output: " + prop.toString());
//                     }
//                     // Skip properties that are not ports.
//                 }
//             }
//         });

//         Object.assign(this, {
//             _disconnectContainedReceivers(port: Port<*>): void {
//                 for (var receivers of relations.values()) {
//                         receivers.delete(port);
//                 }
//             }

//         });

//         Object.assign(this, {
//             _disconnectContainedSource(port: Port<*>): void {
//                 relations.delete(port);
//             }
//         });
    
//         Object.assign(this, {
//             _add(...components: Array<Component>): void {
//                 for (var c of components) {
//                     c._acquire(this);
//                     c._setName(c._getName()); // to ensure proper indexing
                    
//                     // FIXME: is actor, not component actors.add(c);
//                 }
//             }
//         });

//         Object.assign(this, {
//             _getGraph(): string {
//                 var str = "";
//                 relations.forEach(function(val, key, map) {
//                     str += `${key._getFullyQualifiedName()} => ` + "[";
//                     for (var p of val) {
//                         str += p._getFullyQualifiedName() + ", ";
//                     }
//                     str = str.substring(0, str.length-2);
//                     str += "]"
//                 });
//                 return str;
//             }
//         });
//     }


//     /**
//      * Add a list of elements to this container.
//      * @param {T} element
//      */
//     _add: (...components: Array<Component>) => void;
        

//     /**
//      * Return whether or not the argument is present in the container.
//      * @param {T} element
//      */
//     _contains(element: Component): boolean { // FIXME!
//         return true; //this._components.includes(element);
//     }

//     /**
//      * Remove an element from this container.
//      * @param {string} name
//      */
//     _remove(element: Component): void {
//         // check whether it is connected to anything
//         // remove all connections
//     }

// }

// /**
//  * A parameter is an input port that has a default value. 
//  * If no current value is present, get() returns the default value.
//  * Unlike regular input ports, parameters are persisent by default,
//  * which means that their current value only changes when an new
//  * input becomes known _and present_ (i.e., the current value remains
//  * unchanged until the next message arrives). 
//  */
// export class Parameter<T> extends InPort<T> {

//     default:T;
    
//     get: () => T | null;
//     read: () => T;

//     constructor(parent: Reactor, defaultValue:T, persist:boolean=true) {
//         super(parent, persist);
//         this._value = defaultValue; // FIXME: probably put this in the constructor scope
//         // Object.assign(this, {
//         //     send(value: ?$Subtype<T>, delay?:number): void {
//         //         if (value == null) {
//         //             this.reset();
//         //         } else {
//         //             this._default = value; // FIXME: default vs current value
//         //         }
//         //     }
//         // });

//         Object.assign(this, {
//             read(): T {
//                 let val = this.get();
//                 if (val != null) {
//                     return val; 
//                 } else {
//                     return this.default;
//                 }
//             }
//         });
//     }

//     reset() {
//         this._value = this.default;
//     }

// }



/**
 * Base class for reactions that has two type parameters: 
 * T, which describes a tuple of inputs/outputs/actions it may use;
 * S, which describes an object that keeps shared state.
 * The reaction can also maintain state locally.
 */

 // triggeredby/uses/produces
// export class Reaction<T,S:?Object> {

//     io:T
//     shared:S;
    


// // FIXME: need a get/set/schedule here to shadow the global one

//     portsInScope: () => [Set<InPort<mixed>>, Set<OutPort<mixed>>];

//     +react: (time?:number) => void;

//     constructor(io:T, state:S) {
//         this.io = io;
//         this.shared = state;

//         /**
//          * Given some data structure, recursively find all references
//          * to any input and output ports.
//          */
//         function collect(inputs: Set<InPort<mixed>>, 
//             outputs: Set<OutPort<mixed>>, visited: Set<Object>, data:any) {
//             if (data instanceof InPort) {
//                 inputs.add(data);
//             } 
//             else if (data instanceof OutPort) {
//                 outputs.add(data);
//             }
//             else if (data != null && data === Object(data)) {
//                 visited.add(data);
//                 if (typeof data[Symbol.iterator] === 'function') {
//                     // Iterate if iterable
//                     for (let elem of data) {
//                         if (!visited.has(elem))
//                             collect(inputs, outputs, visited, elem);
//                     }
//                 } else {
//                     // Loop over object entries otherwise
//                     for (const [key, value] of Object.entries(data)) {
//                         if (!visited.has(value))
//                             collect(inputs, outputs, visited, value);
//                     }            
//                 }
//             } else {
//                 console.log(data)
//             }
//         }

//         Object.assign(this, {
//             portsInScope(): [Set<InPort<mixed>>, Set<OutPort<mixed>>] {
//                 var inputs = new Set<InPort<mixed>>();
//                 var outputs = new Set<OutPort<mixed>>();
//                 collect(inputs, outputs, new Set<Object>(), this);
//                 return [inputs, outputs];
//             }
//         });
//     }
// }

// export class OrderedAsyncReaction<T, S, R, E> extends Reaction<T, S> {

//     reqID = -1;
//     queue: PriorityQueue<R> = new PriorityQueue();
//     response: Action<R>;
//     error: Action<E>;

//     constructor(io:T, state:S, response:Action<R>, error:Action<E>) {
//         super(io, state);
//         this.response = response;
//         this.error = error;
//     }

//     react(time?: number):void {
        
//         let myID = this.reqID++;
//         // this.queue.push(null, myID); FIXME: find another way to do this
//         (async () => {
//             try {
//                 const response = await this.doAsync();
//                 var firstInLine = this.queue.first();
                
//                 // schedule reactions to preceeding replies
//                 while(firstInLine.value != null && firstInLine.priority < myID) {
//                     this.response.schedule(this.queue.pop()); // NOTE: schedule must pile these up in superdense time!
//                     firstInLine = this.queue.first();
//                 }

//                 if (firstInLine.priority == myID) {
//                     // schedule a reaction to the current reply
//                     this.response.schedule(response);
//                     this.queue.pop();
//                 } else {
//                     //this.queue.update(response, myID); FIXME
//                 }
                
//                 // further empty the queue as much as possible
//                 while(firstInLine.value != null) {
//                     this.response.schedule(this.queue.pop());
//                     firstInLine = this.queue.first();
//                 }
                
//             } catch (err) {
                
//                 // remove corresponding entry from the queue
//                 this.queue.remove(myID);

//                 // schedule a reaction to the error
//                 this.error.schedule(err);

//                 var firstInLine = this.queue.first();
//                 // further empty the queue as much as possible
//                 while(firstInLine.value != null) {
//                     this.response.schedule(this.queue.pop());
//                     firstInLine = this.queue.first();
//                 }
//             }
//         })();
//     }

//     doAsync(): Promise<R> {
//         return new Promise(function(resolve, reject) {});
//     }

// }



// Eventually, this should become a worker/threaded composite
// Also, check out https://github.com/laverdet/isolated-vm





// class Countdown {
//     constructor(counter, action) {
//         Object.assign(this, {
//             dec(): boolean {
//                 return true;
//             }
//         });
//     }
    
//     dec: () => boolean
// }


// const c = new Countdown(2, () => console.log('DONE'));
// c.dec();
// c.dec();


// class FinalClass {
//     constructor(secret) {
//     if (this.constructor !== FinalClass) {
//       throw new Error('Subclassing is not allowed');
//     }
//   }
// }

// class Extension extends FinalClass {

// }

// let y = new Extension();

// var oldProto = FinalClass.prototype;
// FinalClass = function(secret) { console.log(secret)};
// FinalClass.prototype = oldProto;

// let z = new FinalClass("do not read this");





// Scenario 1:
// The composite reacts to inputs.
// - set the inputs of the receivers
// - let them react in dependency order

// *** what if there is a delay?
// - 


// Scenario 2:
// An actor spontaneously emits an event


// datastructures:
// - dependency graph


// - calendarQ t -> [], where events are sorted by priority/index
// types of events:
// - self-scheduled
// - dataflow (from other actors)
// *** what about RMI?
// - the schedule must ensure that upon invocation all inputs are known
// - the invocation itself must be a call similar to send(), except it has to function like a procedure call (do we need two stacks?)
//   - before a remote procedure can yield/return, all of the inputs it uses must be known
//   - reactions within the same actor must be mutually atomic, across actors this need not be the case
// *** how are reactions and RPCs different?
//   - RPCs are reactions that are triggered by an event on a CalleePort<A,R>
//   - an RPC port is special because it has an argument type and a return type
//   - the return value must be set by the callee
// *** what if there are multiple callers?
//   - this is similar to the problem of multiple senders; a reaction will take place for each distinct caller/sender
//   - if RPC's can modify state, the order of invocation matters (dependencies must be introduced between the callers)
// *** what if there are multiple calls from the same caller?
//   - this would only be useful if RPCs can modify state, or else subsequent calls will yield the same result
// *** should RPC's be allowed to modify state?
//   - not sure, but if we disallow it, how can we enforce it? Compile error?
// RPC: pull, other than reactive, which is push






// class TransferValue<T> implements UnorderedReaction {
    
//     react(from:Readable<T>, to:Writable<T>) {
//         to.set(from.get());
//     }

// }

// class ActivationRecord<R extends Reaction,A> {
//     reaction:R;
//     args:A;
//     constructor(reaction:R, args:A) {
//         this.reaction = reaction;
//         this.args = args;
//     }
// }