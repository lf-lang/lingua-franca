/* A helper class that represents of a time value. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy

import java.math.BigInteger
import org.icyphy.linguaFranca.TimeUnit

/**
 * A helper class that represents of a time value.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class TimeValue {
    
    /**
     * Big integer representation of this time value in nanoseconds.
     */
    var BigInteger nanoSecs
    
    /**
     * Primitive numerical representation of this time value, to be interpreted
     * in terms the associated time unit.
     */
    public long time = 0L
    
    /**
     * Units associated with this time value.
     */
    public TimeUnit unit = TimeUnit.NONE
    
    /**
     * Factor for turning microseconds into nanoseconds.
     */
    static var US = new BigInteger("1000")

    /**
     * Factor for turning milliseconds into nanoseconds.
     */
    static var MS = new BigInteger("1000000")
    
    /**
     * Factor for turning seconds into nanoseconds.
     */
    static var S = new BigInteger("1000000000")
    
    /**
     * Factor for turning minutes into nanoseconds.
     */
    static var M = new BigInteger("60000000000")
    
    /**
     * Factor for turning hours into nanoseconds.
     */
    static var H = new BigInteger("3600000000000")
    
    /**
     * Factor for turning days into nanoseconds.
     */
    static var D = new BigInteger("86400000000000")
    
    /**
     * Factor for turning weeks into nanoseconds.
     */
    static var W = new BigInteger("604800016558522")

    /**
     * Maximum size of a deadline in primitive representation.
     * NOTE: if we were to use an unsigned data type the this would be
     * 0xFFFFFFFFFFFF
     */
    public static long MAX_LONG_DEADLINE = Long.decode("0x7FFFFFFFFFFF")

    /**
     * Maximum size of a deadline in big integer representation.
     * NOTE: if we were to use an unsigned data type the this would be
     * 0xFFFFFFFFFFFF
     */
    public static BigInteger MAX_BIGINT_DEADLINE = 
            new BigInteger("7FFFFFFFFFFF", 16)
    
    /**
     * Create a new time value. Throws an exception time is non-zero and no
     * units are given.
     */
    new(long time, TimeUnit unit) {
        this.time = time
        this.unit = unit
        
        val bigTime = new BigInteger(this.time.toString)
        
        switch(this.unit) {
            case TimeUnit.NONE : {
                if (time != 0) {
                    throw new Error("Non-zero time values must have a unit.")
                }
                this.nanoSecs = bigTime
            }
            case TimeUnit.NSEC,
            case TimeUnit.NSECS:
                this.nanoSecs = bigTime
            case TimeUnit.USEC,
            case TimeUnit.USECS:
                this.nanoSecs = bigTime.multiply(US)
            case TimeUnit.MSEC,
            case TimeUnit.MSECS:
                this.nanoSecs = bigTime.multiply(MS)
            case TimeUnit.SEC,
            case TimeUnit.SECS,
            case TimeUnit.SECOND,
            case TimeUnit.SECONDS:
                this.nanoSecs = bigTime.multiply(S)
            case TimeUnit.MIN,
            case TimeUnit.MINS,
            case TimeUnit.MINUTE,
            case TimeUnit.MINUTES:
                this.nanoSecs = bigTime.multiply(M)
            case TimeUnit.HOURS,
            case TimeUnit.HOUR:
                this.nanoSecs = bigTime.multiply(H)
            case TimeUnit.DAY,
            case TimeUnit.DAYS:
                this.nanoSecs = bigTime.multiply(D)
            case TimeUnit.WEEK,
            case TimeUnit.WEEKS:
                this.nanoSecs = bigTime.multiply(W)
                
        }
        
    }
        
    /**
     * Determine whether this time value is earlier than another.
     * @returns true if this time value is earlier than the other, false
     * otherwise.
     */
    def isEarlierThan(TimeValue other) {
        if (this.toNanoSeconds < other.toNanoSeconds)
            true
        else
            false
    }
    
    /**
     * Get this time value in number of nanoseconds.
     */
    def BigInteger toNanoSeconds() {
        return this.nanoSecs
    }
    
    /**
     * Return a string representation of this time value.
     */
    override String toString() {
        '''«this.time» «this.unit»'''
    }
    
    /**
     * Map from time units to an expression that can convert a number in
     * the specified time unit into nanoseconds. This expression may need
     * to have a suffix like 'LL' or 'L' appended to it, depending on the
     * target language, to ensure that the result is a 64-bit long.
     */
    public static val timeUnitsToNs = #{TimeUnit.NSEC -> 1L,
        TimeUnit.NSECS -> 1L, TimeUnit.USEC -> 1000L, TimeUnit.USECS -> 1000L,
        TimeUnit.MSEC -> 1000000L, TimeUnit.MSECS -> 1000000L,
        TimeUnit.SEC -> 1000000000L, TimeUnit.SECS -> 1000000000L,
        TimeUnit.SECOND -> 1000000000L, TimeUnit.SECONDS -> 1000000000L,
        TimeUnit.MIN -> 60000000000L, TimeUnit.MINS -> 60000000000L,
        TimeUnit.MINUTE -> 60000000000L, TimeUnit.MINUTES -> 60000000000L,
        TimeUnit.HOUR -> 3600000000000L, TimeUnit.HOURS -> 3600000000000L,
        TimeUnit.DAY -> 86400000000000L, TimeUnit.DAYS -> 86400000000000L,
        TimeUnit.WEEK -> 604800000000000L, TimeUnit.WEEKS -> 604800000000000L}
        
    public static TimeValue MAX_VALUE = new TimeValue(Long.MAX_VALUE, TimeUnit.WEEKS)
    public static TimeValue MIN_VALUE = new TimeValue(Long.MIN_VALUE, TimeUnit.WEEKS)
    
    /**
     * Return the result of adding b's time to the calling
     * object's time.
     * 
     * The unit of the returned TimeValue will be the minimum 
     * of the units of both operands except if only one of the units 
     * is TimeUnit.NONE. In that case, the unit of the other input is used.
     * 
     * @input b The right operand
     * @return A new TimeValue (the current value will not be affected) 
     */
    def TimeValue add(TimeValue b) {
        // Figure out the actual sum
        var sumOfNumbers = this.toNanoSeconds() + b.toNanoSeconds()
        // Unit of the return
        var TimeUnit returnUnit
        // Compare this unit against b's unit and use the smallest value
        val isThisUnitSmallerThanBUnit = this.unit.compareTo(b.unit) <= 0
        returnUnit = (isThisUnitSmallerThanBUnit) ? this.unit : b.unit
        // In the corner case where one of the TimeUnits is NONE
        if (returnUnit == TimeUnit.NONE) {
            // Check if both TimeUnits are NONE
            if (b.unit == TimeUnit.NONE && this.unit == TimeUnit.NONE) {
                // Since both time units are NONE, the sum should be 0
                if (sumOfNumbers != 0) {
                    // Double-check to ensure the logic is correct
                    throw new Error("Adding two TimeValues failed: Non-zero time values must have a unit.")
                }
                return new TimeValue(0, TimeUnit.NONE)                
            } 
            // Only one of the units is NONE
            // Use the maximum of the units instead            
            returnUnit = (!isThisUnitSmallerThanBUnit) ? this.unit : b.unit
        }
        // Find the appropriate divider to bring sumOfNumbers from nanoseconds to returnUnit
        val unitDivider = new BigInteger(timeUnitsToNs.get(returnUnit).toString)        
        return new TimeValue(sumOfNumbers.divide(unitDivider).longValue, returnUnit)
    }
    
}
