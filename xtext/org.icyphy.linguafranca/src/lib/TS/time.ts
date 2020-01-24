/**
 * Time-related helper functions for reactors.
 * @author Marten Lohstroh (marten@berkeley.edu)
 * @author Matt Weber (matt.weber@berkeley.edu)
 */

/**
 * Module used to acquire time from the platform in microsecond precision.
 * @see {@link https://www.npmjs.com/package/microtime}
 */
const Microtime = require("microtime");

/**
 * Units (and conversion factors from nanoseconds) for time values.
 **/
export enum TimeUnit {
    nsec = 1,
    usec = 1000,
    msec = 1000000,
    sec = 1000000000,
    secs = 1000000000,
    minute = 60000000000,
    minutes = 60000000000,
    hour = 3600000000000,
    hours = 3600000000000,
    day = 86400000000000,
    days = 86400000000000,
    week = 604800000000000,
    weeks = 604800000000000
}

/**
 * A time interval given in nanosecond precision. To prevent overflow
 * (which would occur for time intervals spanning more than 0.29 years
 * if a single JavaScript number, which has 2^53 bits of precision, were
 * to be used), we use _two_ numbers to store a time interval. The first
 * number denotes the number of whole seconds in the interval; the second
 * number denotes the remaining number of nanoseconds in the interval.
 * This class serves as a base class for `UnitBasedTimeInterval`, which 
 * provides the convenience of defining time intervals as a single number
 * accompanied by a unit.
 * @see TimeUnit
 * @see UnitBasedTimeInterval
 */
export class TimeInterval {

    /**
     * Create a new time interval. Both parameters must be non-zero integers;
     * an error will be thrown otherwise. The second parameter is optional.
     * @param seconds Number of seconds in the interval.
     * @param nanoseconds Remaining number of nanoseconds (defaults to zero).
     */
    constructor(protected seconds: number, protected nanoseconds: number=0) {
        if(!Number.isInteger(seconds) || !Number.isInteger(nanoseconds) || seconds < 0 || nanoseconds < 0) {
            throw new Error("Cannot instantiate a time interval based on negative or non-integer numbers.");
        }
    }

    /**
     * Return a new time interval that denotes the duration of this 
     * time interval plus the time interval given as a parameter.
     * @param other The time interval to add to this one.
     */
    add(other: TimeInterval): TimeInterval {
        const billion = 1000000000;

        let seconds = this.seconds + other.seconds;
        let nanoseconds = this.nanoseconds + other.nanoseconds;

        if(nanoseconds >= billion) {
            // Carry the second.
            seconds += 1;
            nanoseconds -= billion;
        }
        return new TimeInterval(seconds, nanoseconds);
    }

    /**
     * Return a new time interval that denotes the duration of this 
     * time interval minus the time interval given as a parameter.
     * @param other The time interval to subtract from this one.
     */
    subtract(other: TimeInterval): TimeInterval {
        var s = this.seconds - other.seconds;;
        var ns = this.nanoseconds - other.nanoseconds;

        if(ns < 0) {
            // Borrow a second
            s -= 1;
            ns += 1000000000;
        }

        if(s < 0){
            throw new Error("Negative time value.");
        }
        return new TimeInterval(s, ns);
    }

    /**
     * Return true this denotes a time interval equal of equal
     * length as the time interval given as a parameter.
     * @param other The time interval to compare to this one.
     */
    isEqualTo(other: TimeInterval): boolean {
        return this.seconds == other.seconds 
            && this.nanoseconds == other.nanoseconds;
    }

    /**
     * Return true if this denotes a zero time interval.
     */
    isZero() {
        if(this.seconds == 0 && this.nanoseconds == 0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Return true if this time interval is of smaller length than the time
     * interval given as a parameter, return false otherwise.
     * NOTE: Performing this comparison involves a conversion to a big integer
     * and is therefore relatively costly.
     * @param other The time interval to compare to this one.
     * @see {@link https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/BigInt|BigInt} for further information.
     */
    isSmallerThan(other: TimeInterval) {
        if (this.seconds < other.seconds) {
            return true;
        }
        if (this.seconds == other.seconds && this.nanoseconds < other.nanoseconds) {
            return true;
        }
        return false;
    }

    /**
     * Print the number of seconds and nanoseconds in this time interval.
     */
    public toString(): string {
        return "(" + this.seconds + " secs; " + this.nanoseconds + " nsecs)";
    }

    /**
     * Get a string representation of this time interval that is compatible
     * with `nanotimer`. Unit specifiers are `s` for seconds, `m` for 
     * milliseconds `u` for microseconds, and `n` for nanoseconds.
     * @see {@link https://www.npmjs.com/package/nanotimer} for 
     * further information.
     */
    public getNanoTime(): string {
        
        if (this.nanoseconds == 0) {
            // Seconds.
            return this.seconds.toString() + "s";
        } else if (this.nanoseconds % 1000000 == 0) {
            // Milliseconds.
            let msecs = (this.nanoseconds / 1000000).toString();
            if (this.seconds == 0) {
                return msecs + "m";
            } else {
                let padding = "";
                for (let i = 0; i < 3 - msecs.length; i++) {
                    padding += "0";
                }
                return this.seconds.toString() + padding + msecs + "m";
            }
        } else if (this.nanoseconds % 1000 == 0) {
            // Microseconds.
            let usecs = (this.nanoseconds/1000).toString();
            if (this.seconds == 0) {
                return usecs + "u";
            } else {
                let padding = "";
                for (let i = 0; i < 6 - usecs.length; i++) {
                    padding += "0";
                }
                return this.seconds.toString() + padding + usecs + "u";
            }
        } else {
            // Nanoseconds.
            if (this.seconds == 0) {
                return this.nanoseconds + "n";
            } else {
                let nsecs = this.nanoseconds.toString();
                let padding = "";
                for (let i = 0; i < 9 - nsecs.length; i++) {
                    padding += "0";
                }
                return this.seconds.toString() + padding + nsecs + "n";
            }
        } 
    }
}

/** 
 * Subclass of `TimeInterval` that is constructed on the basis of a value
 * accompanied with a time unit. The value is a `number` that is required to be
 * a positive integer. The time unit must be a member of the `TimeUnit` enum.
 */
export class UnitBasedTimeInterval extends TimeInterval {

    /**
     * Construct a new time interval on the basis of a value accompanied with
     * a time unit. An error is thrown when the given value is negative, 
     * non-integer, or both.
     * @param value A number (which must be a positive integer) that denotes 
     * the length of the specified time interval, expressed as a multiple of
     * the given time unit.
     * @param unit The unit of measurement that applies to the given value.
     */
    constructor(private value: number, private unit:TimeUnit) {
        super(0, 0); 

        if (!Number.isInteger(value)) {
            throw new Error("Non-integer time values are illegal.");
        }
        if (value < 0) {
            throw new Error("Negative time values are illegal.");
        }
        
        const billion = BigInt(TimeUnit.secs);
        
        // To avoid overflow and floating point errors, work with BigInts.
        let bigT = BigInt(this.value) * BigInt(this.unit);  
        let bigSeconds = bigT / billion;
        
        if(bigSeconds > Number.MAX_SAFE_INTEGER) {
            throw new Error("Unable to instantiate time interval: value too large.");
        }
        
        this.seconds = Number(bigSeconds);
        this.nanoseconds = Number(bigT % billion);
    }

    /**
     * Print a string representation of this time interval using the time unit it was
     * originally created with.
     */
    public toString(): string {
        return this.value + " " + TimeUnit[this.unit];
    }
}


/** 
 * A superdense time instant, represented as a time interval `time` (i.e., 
 * time elapsed since Epoch) paired with a microstep index to keep track of 
 * iterations in superdense time. For each such iteration, `time` remains
 * the same, but `microstep` is incremented. 
 */ 
export class TimeInstant {

    /**
     * Time elapsed since Epoch.
     */
    readonly time:TimeInterval;

    /**
     * 
     * @param timeSinceEpoch Time elapsed since Epoch.
     * @param microstep Superdense time index.
     */
    constructor(timeSinceEpoch: TimeInterval, readonly microstep: number=0) {
        if (!Number.isInteger(microstep)) {
            throw new Error("Microstep must be integer.");
        }
        if (microstep < 0) {
            throw new Error("Microstep must be positive.");
        }
        this.time = timeSinceEpoch;
    }

    /**
     * Return `true` if this time instant is earlier than the time instant
     * given as a parameter, false otherwise. For two time instants with
     * an equal `time`, one instant is earlier than the other if its 
     * `microstep` is less than the `microstep` of the other.
     * @param other The time instant to compare against this one.
     */
    isEarlierThan(other: TimeInstant): boolean {
        return this.time.isSmallerThan(other.time) 
            || (this.time.isEqualTo(other.time) 
                && this.microstep < other.microstep);
    }

    /**
     * Return `true` if this time instant is simultaneous with the time
     * instant given as a parameter, false otherwise. Both `time` and 
     * `microstep` must be equal for two time instants to be simultaneous.
     * @param other The time instant to compare against this one.
     */
    isSimultaneousWith(other: TimeInstant) {
        return this.time.isEqualTo(other.time) 
            && this.microstep == other.microstep;
    }
    
    /**
     * Get a new time instant that represents this time instant plus
     * the given delay. The `microstep` of this time instant is ignored;
     * the returned time instant always has a `microstep` of zero.
     * @param delay The time interval to add to this time instant.
     */
    getLaterTime(delay: TimeInterval) : TimeInstant {
        return new TimeInstant(delay.add(this.time), 0);
    }

    /**
     * Get a new time instant that has the same `time` but a `microstep` that
     * is incremented by one relative to this time instant's `microstep`.
     */
    getMicroStepLater() {
        return new TimeInstant(this.time, this.microstep+1);
    }

    /**
     * Obtain a time interval that represents the absolute (i.e., positive)
     * time difference between this time interval and the time interval given
     * as a parameter.
     * @param other The time instant for which to compute the absolute difference
     * with this time instant.
     * 
     */
    getTimeDifference(other: TimeInstant): TimeInterval {
        if (this.isEarlierThan(other)) {
            return other.time.subtract(this.time);
        } else {
            return this.time.subtract(other.time);
        }
    }

    /**
     * Return a human-readable string presentation of this time instant.
     */
    public toString(): string {
        return "(" + this.time.toString() + ", " + this.microstep + ")"; 
    }
}

/**
 * A descriptor to be used when scheduling events to denote as to whether
 * an event should occur with a delay relative to _physical time_ or _logical
 * time_.
 */
export enum Origin {
    physical,
    logical
}

/**
 * Return a time instant that reflects the current physical time as reported
 * by the platform.
 */
export function getCurrentPhysicalTime(): TimeInstant {
    let t = Microtime.now();
    let seconds: number = Math.floor(t / 1000000);
    let nseconds: number = t * 1000 - seconds * 1000000000;
    return new TimeInstant(new TimeInterval(seconds, nseconds), 0);
}
