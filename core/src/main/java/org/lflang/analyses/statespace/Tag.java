package org.lflang.analyses.statespace;

import org.lflang.TimeValue;

/**
 * Class representing a logical time tag, which is a pair that consists of a timestamp (type long)
 * and a microstep (type long).
 */
public class Tag implements Comparable<Tag> {

  public final long timestamp;
  public final long microstep;
  public final boolean forever; // Whether the tag is FOREVER into the future.

  public Tag(long timestamp, long microstep, boolean forever) {
    this.timestamp = timestamp;
    this.microstep = microstep;
    this.forever = forever;
  }

  @Override
  public int compareTo(Tag t) {
    // If one tag is forever, and the other is not,
    // then forever tag is later. If both tags are
    // forever, then they are equal.
    if (this.forever && !t.forever) return 1;
    else if (!this.forever && t.forever) return -1;
    else if (this.forever && t.forever) return 0;

    // Compare the timestamps if they are not equal.
    if (this.timestamp != t.timestamp) {
      if (this.timestamp > t.timestamp) return 1;
      else if (this.timestamp < t.timestamp) return -1;
      else return 0;
    }

    // Otherwise, compare the microsteps.
    if (this.microstep > t.microstep) return 1;
    else if (this.microstep < t.microstep) return -1;
    else return 0;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o instanceof Tag) {
      Tag t = (Tag) o;
      if (this.timestamp == t.timestamp
          && this.microstep == t.microstep
          && this.forever == t.forever) return true;
    }
    return false;
  }

  @Override
  public String toString() {
    if (this.forever) return "(FOREVER, " + this.microstep + ")";
    else {
      TimeValue time = TimeValue.fromNanoSeconds(this.timestamp);
      return "(" + time + ", " + this.microstep + ")";
    }
  }
}
