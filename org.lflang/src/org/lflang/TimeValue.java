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

package org.lflang;

import org.lflang.lf.TimeUnit;

/**
 * Represents an amount of time (a duration).
 *
 * @author Marten Lohstroh <marten@berkeley.edu>
 */
public final class TimeValue implements Comparable<TimeValue> {

    /**
     * The maximum value of this type. This is approximately equal to 292 years.
     */
    public static final TimeValue MAX_VALUE = new TimeValue(Long.MAX_VALUE, TimeUnit.NSECS);

    /**
     * A time value equal to zero.
     */
    public static final TimeValue ZERO = new TimeValue(0, TimeUnit.NONE);

    /**
     * Primitive numerical representation of this time value,
     * to be interpreted in terms the associated time unit.
     */
    public final long time;

    /**
     * Units associated with this time value.
     */
    public final TimeUnit unit;

    /**
     * Maximum size of a deadline in primitive representation.
     * NOTE: if we were to use an unsigned data type this would be
     * 0xFFFFFFFFFFFF
     */
    public static final long MAX_LONG_DEADLINE = Long.decode("0x7FFFFFFFFFFF");

    /**
     * Create a new time value. Throws an exception time is non-zero and no
     * units are given.
     */
    public TimeValue(long time, TimeUnit unit) {
        if (unit == TimeUnit.NONE && time != 0) {
            throw new IllegalArgumentException("Non-zero time values must have a unit");
        }
        this.time = time;
        this.unit = unit;
    }

    private static long makeNanosecs(long time, TimeUnit unit) {
        switch (unit) {
        case NONE:
            return time; // == 0, see constructor.
        case NSEC:
        case NSECS:
            return time;
        case USEC:
        case USECS:
            return time * 1000;
        case MSEC:
        case MSECS:
            return time * 1_000_000;
        case SEC:
        case SECS:
        case SECOND:
        case SECONDS:
            return time * 1_000_000_000;
        case MIN:
        case MINS:
        case MINUTE:
        case MINUTES:
            return time * 60_000_000_000L;
        case HOURS:
        case HOUR:
            return time * 3_600_000_000_000L;
        case DAY:
        case DAYS:
            return time * 86_400_000_000_000L;
        case WEEK:
        case WEEKS:
            return time * 604_800_016_558_522L;
        }
        throw new AssertionError("unreachable");
    }

    /**
     * Returns whether this time value is earlier than another.
     */
    public boolean isEarlierThan(TimeValue other) {
        return this.compareTo(other) < 0;
    }

    @Override
    public int compareTo(TimeValue o) {
        if (o == null) { 
            throw new NullPointerException("Can't compare against null timevalue");
        }
        return Long.compare(this.toNanoSeconds(), o.toNanoSeconds());
    }

    /**
     * Get this time value in number of nanoseconds.
     */
    public long toNanoSeconds() {
        return makeNanosecs(time, unit);
    }

    /**
     * Return a string representation of this time value.
     */
    public String toString() {
        return unit != TimeUnit.NONE ? time + " " + unit
                                     : Long.toString(time);
    }

    /** Return the latest of both values. */
    public static TimeValue max(TimeValue t1, TimeValue t2) {
        return t1.isEarlierThan(t2) ? t2 : t1;
    }

    /**
     * Return the sum of this duration and the one represented by b.
     * <p>
     * The unit of the returned TimeValue will be the minimum
     * of the units of both operands except if only one of the units
     * is TimeUnit.NONE. In that case, the unit of the other input is used.
     *
     * @param b The right operand
     * @return A new TimeValue (the current value will not be affected)
     */
    public TimeValue add(TimeValue b) {
        // Figure out the actual sum
        final long sumOfNumbers;
        try {
            sumOfNumbers = Math.addExact(this.toNanoSeconds(), b.toNanoSeconds());
        } catch (ArithmeticException overflow) {
            return MAX_VALUE;
        }

        if (this.unit == TimeUnit.NONE || b.unit == TimeUnit.NONE) {
            // A time value with no unit is necessarily zero. So
            // if this is none, (this + b) == b, if b is none, (this+b) == this.
            return b.unit == TimeUnit.NONE ? this : b;
        }
        boolean isThisUnitSmallerThanBUnit = this.unit.compareTo(b.unit) <= 0;
        TimeUnit smallestUnit = isThisUnitSmallerThanBUnit ? this.unit : b.unit;
        // Find the appropriate divider to bring sumOfNumbers from nanoseconds to returnUnit
        var unitDivider = makeNanosecs(1, smallestUnit);
        return new TimeValue(sumOfNumbers / unitDivider, smallestUnit);
    }

}
