package org.icyphy

import java.math.BigInteger
import org.icyphy.linguaFranca.TimeUnit

class TimeValue {
    public long time = 0L
    public TimeUnit unit = TimeUnit.NONE

    var BigInteger bigTime
    static var US = new BigInteger("1000")
    static var MS = new BigInteger("1000000")
    static var S = new BigInteger("1000000000")
    static var M = new BigInteger("60000000000")
    static var H = new BigInteger("3600000000000")
    static var D = new BigInteger("86400000000000")
    static var W = new BigInteger("604800016558522")

    /**
     * NOTE: if we were to use an unsigned data type the this would be
     * 0xFFFFFFFFFFFF
     */
    public static long MAX_LONG_DEADLINE = Long.decode("0x7FFFFFFFFFFF")

    public static BigInteger MAX_BIGINT_DEADLINE = new BigInteger("7FFFFFFFFFFF", 16)
    
    new(long time, TimeUnit unit) {
        this.time = time
        this.unit = unit
        this.bigTime = new BigInteger(this.time.toString)
    }
    
    def BigInteger toNanoSeconds() {
        switch(this.unit) {
            case TimeUnit.NONE : 
                throw new Error("Time values must have a unit.")
            case TimeUnit.NSEC,
            case TimeUnit.NSECS:
                return this.bigTime
            case TimeUnit.USEC,
            case TimeUnit.USECS:
                return this.bigTime.multiply(US)
            case TimeUnit.MSEC,
            case TimeUnit.MSECS:
                return this.bigTime.multiply(MS)
            case TimeUnit.SEC,
            case TimeUnit.SECS,
            case TimeUnit.SECOND,
            case TimeUnit.SECONDS:
                return this.bigTime.multiply(S)
            case TimeUnit.MIN,
            case TimeUnit.MINS,
            case TimeUnit.MINUTE,
            case TimeUnit.MINUTES:
                return this.bigTime.multiply(M)
            case TimeUnit.HOURS,
            case TimeUnit.HOUR:
                return this.bigTime.multiply(H)
            case TimeUnit.DAY,
            case TimeUnit.DAYS:
                return this.bigTime.multiply(D)
            case TimeUnit.WEEK,
            case TimeUnit.WEEKS:
                return this.bigTime.multiply(W)
                
        }
    }
    
    def isEarlierThan(TimeValue other) {
        if (this.toNanoSeconds < other.toNanoSeconds)
            true
        else
            false
    }
}
