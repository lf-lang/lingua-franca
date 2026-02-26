package org.lflang;

import org.lflang.lf.Time;

/**
 * Represents an amount of time (a duration).
 *
 * @author Marten Lohstroh
 * @author Clément Fournier - TU Dresden, INSA Rennes
 * @ingroup Utilities
 */
public final class TimeValue implements Comparable<TimeValue> {

  /** The maximum value of this type. This is approximately equal to 292 years. */
  public static final TimeValue MAX_VALUE = new TimeValue(Long.MAX_VALUE, TimeUnit.NANO);

  /** The minimum value of this type. */
  public static final TimeValue MIN_VALUE = new TimeValue(Long.MIN_VALUE, TimeUnit.NANO);

  /** A time value equal to zero. */
  public static final TimeValue ZERO = new TimeValue(0, null);

  /** A time value representing NEVER, which is less than any other time value. */
  public static final TimeValue NEVER = new TimeValue(Long.MIN_VALUE, TimeUnit.NANO);

  /** A time value representing FOREVER which is greater than any other time value. */
  public static final TimeValue FOREVER = new TimeValue(Long.MAX_VALUE, TimeUnit.NANO);

  /**
   * Primitive numerical representation of this time value, to be interpreted in terms the
   * associated time unit.
   */
  public final long time;

  /** Units associated with this time value. May be null. */
  public final TimeUnit unit;

  /**
   * Maximum size of a deadline in primitive representation. NOTE: if we were to use an unsigned
   * data type this would be 0xFFFFFFFFFFFF
   */
  public static final long MAX_LONG_DEADLINE = Long.decode("0x7FFFFFFFFFFF");

  /**
   * Create a new time value.
   *
   * @param time The time value.
   * @param unit The unit of the time value.
   * @throws IllegalArgumentException If time is non-zero and the unit is null.
   */
  public TimeValue(long time, TimeUnit unit) {
    if (unit == null && time != 0) {
      throw new IllegalArgumentException("Non-zero time values must have a unit");
    }
    this.time = time;
    this.unit = unit;
  }

  /**
   * Create a new time value.
   *
   * @param time The time AST node..
   * @throws IllegalArgumentException If time is non-zero and the unit is null.
   */
  public TimeValue(Time time) {
    if (time == null || time.getNever() != null) {
      this.time = Long.MIN_VALUE;
      this.unit = TimeUnit.NANO;
    } else if (time.getForever() != null) {
      this.time = Long.MAX_VALUE;
      this.unit = TimeUnit.NANO;
    } else {
      this.time = time.getInterval();
      this.unit = TimeUnit.fromName(time.getUnit());
    }
  }

  @Override
  public boolean equals(Object t1) {
    if (t1 instanceof TimeValue) {
      return this.compareTo((TimeValue) t1) == 0;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return (int) this.toNanoSeconds();
  }

  public static int compare(TimeValue t1, TimeValue t2) {
    if (t1.isEarlierThan(t2)) {
      return -1;
    }
    if (t2.isEarlierThan(t1)) {
      return 1;
    }
    return 0;
  }

  private static long makeNanosecs(long time, TimeUnit unit) {
    if (unit == null) {
      return time; // == 0, see constructor.
    }
    switch (unit) {
      case NANO:
        return time;
      case MICRO:
        return time * 1000;
      case MILLI:
        return time * 1_000_000;
      case SECOND:
        return time * 1_000_000_000;
      case MINUTE:
        return time * 60_000_000_000L;
      case HOUR:
        return time * 3_600_000_000_000L;
      case DAY:
        return time * 86_400_000_000_000L;
      case WEEK:
        return time * 604_800_016_558_522L;
    }
    throw new AssertionError("unreachable");
  }

  /** Returns whether this time value is earlier than another. */
  public boolean isEarlierThan(TimeValue other) {
    return this.compareTo(other) < 0;
  }

  @Override
  public int compareTo(TimeValue o) {
    return Long.compare(this.toNanoSeconds(), o.toNanoSeconds());
  }

  /**
   * Return the magnitude of this value, as expressed in the {@linkplain #getUnit() unit} of this
   * value.
   */
  public long getMagnitude() {
    return time;
  }

  /** Units associated with this time value. May be null, but only if the magnitude is zero. */
  public TimeUnit getUnit() {
    return unit;
  }

  /** Get this time value in number of nanoseconds. */
  public long toNanoSeconds() {
    return makeNanosecs(time, unit);
  }

  /** Return a TimeValue based on a nanosecond value. */
  public static TimeValue fromNanoSeconds(long ns) {
    if (ns == 0) return ZERO;
    long time;
    if ((time = ns / 604_800_016_558_522L) > 0 && ns % 604_800_016_558_522L == 0) {
      return new TimeValue(time, TimeUnit.WEEK);
    }
    if ((time = ns / 86_400_000_000_000L) > 0 && ns % 86_400_000_000_000L == 0) {
      return new TimeValue(time, TimeUnit.DAY);
    }
    if ((time = ns / 3_600_000_000_000L) > 0 && ns % 3_600_000_000_000L == 0) {
      return new TimeValue(time, TimeUnit.HOUR);
    }
    if ((time = ns / 60_000_000_000L) > 0 && ns % 60_000_000_000L == 0) {
      return new TimeValue(time, TimeUnit.MINUTE);
    }
    if ((time = ns / 1_000_000_000) > 0 && ns % 1_000_000_000 == 0) {
      return new TimeValue(time, TimeUnit.SECOND);
    }
    if ((time = ns / 1_000_000) > 0 && ns % 1_000_000 == 0) {
      return new TimeValue(time, TimeUnit.MILLI);
    }
    if ((time = ns / 1000) > 0 && ns % 1000 == 0) {
      return new TimeValue(time, TimeUnit.MICRO);
    }
    return new TimeValue(ns, TimeUnit.NANO);
  }

  /** Return a string representation of this time value. */
  public String toString() {
    if (this.equals(MAX_VALUE)) return "forever";
    if (this.equals(MIN_VALUE)) return "never";
    return unit != null ? time + " " + unit.toString() : Long.toString(time);
  }

  /** Return the latest of both values. */
  public static TimeValue max(TimeValue t1, TimeValue t2) {
    return t1.isEarlierThan(t2) ? t2 : t1;
  }

  /** Return the earliest of both values. */
  public static TimeValue min(TimeValue t1, TimeValue t2) {
    return t1.isEarlierThan(t2) ? t1 : t2;
  }

  /**
   * Return the sum of this duration and the one represented by b.
   *
   * <p>The unit of the returned TimeValue will be the minimum of the units of both operands except
   * if only one of the units is TimeUnit.NONE. In that case, the unit of the other input is used.
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

    if (this.unit == null || b.unit == null) {
      // A time value with no unit is necessarily zero. So
      // if this is null, (this + b) == b, if b is null, (this+b) == this.
      return b.unit == null ? this : b;
    }
    boolean isThisUnitSmallerThanBUnit = this.unit.compareTo(b.unit) <= 0;
    TimeUnit smallestUnit = isThisUnitSmallerThanBUnit ? this.unit : b.unit;
    // Find the appropriate divider to bring sumOfNumbers from nanoseconds to returnUnit
    var unitDivider = makeNanosecs(1, smallestUnit);
    return new TimeValue(sumOfNumbers / unitDivider, smallestUnit);
  }

  /**
   * Return this time value minus the specified time value but no less than 0.
   *
   * <p>The unit of the returned TimeValue will be the minimum of the units of both operands except
   * if only one of the units is TimeUnit.NONE. In that case, the unit of the other input is used.
   *
   * @param b The right operand
   * @return A new TimeValue (the current value will not be affected)
   */
  public TimeValue subtract(TimeValue b) {
    // Figure out the actual difference
    final long differenceOfNumbers;
    try {
      differenceOfNumbers = Math.subtractExact(this.toNanoSeconds(), b.toNanoSeconds());
    } catch (ArithmeticException overflow) {
      return ZERO;
    }
    if (differenceOfNumbers < 0) return ZERO;

    if (this.unit == null || b.unit == null) {
      // A time value with no unit is necessarily zero. So
      // if this is null, (this - b) == ZERO, if b is null, (this - b) == this.
      return b.unit == null ? this : ZERO;
    }
    boolean isThisUnitSmallerThanBUnit = this.unit.compareTo(b.unit) <= 0;
    TimeUnit smallestUnit = isThisUnitSmallerThanBUnit ? this.unit : b.unit;
    // Find the appropriate divider to bring sumOfNumbers from nanoseconds to returnUnit
    var unitDivider = makeNanosecs(1, smallestUnit);
    return new TimeValue(differenceOfNumbers / unitDivider, smallestUnit);
  }
}
