package org.lflang;

/**
 * Class representing a logical time tag, which is a pair that consists of a timestamp and a
 * microstep.
 */
public class TimeTag implements Comparable<TimeTag> {

  public final TimeValue time;
  public final Long microstep;

  /** Constructor */
  public TimeTag(TimeValue time, Long microstep) {
    this.time = time;
    this.microstep = microstep;
  }

  /** Copy constructor */
  public TimeTag(TimeTag that) {
    this.time = that.time;
    this.microstep = that.microstep;
  }

  /**
   * When comparing two time tags, first compare their time fields. If they are equal, compare their
   * microsteps.
   */
  @Override
  public int compareTo(TimeTag t) {
    if (this.time.compareTo(t.time) < 0) return -1;
    else if (this.time.compareTo(t.time) > 0) return 1;
    else return this.microstep.compareTo(t.microstep);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o instanceof TimeTag t && this.compareTo(t) == 0) return true;
    return false;
  }

  @Override
  public String toString() {
    return "( " + this.time + ", " + this.microstep + " )";
  }
}
