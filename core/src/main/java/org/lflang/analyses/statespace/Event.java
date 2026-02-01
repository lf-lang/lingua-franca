package org.lflang.analyses.statespace;

import org.lflang.TimeTag;
import org.lflang.generator.TriggerInstance;

/** A class representing a tagged signal, for analytical purposes */
public class Event implements Comparable<Event> {

  private final TriggerInstance<?> trigger;
  private TimeTag tag;

  public Event(TriggerInstance trigger, TimeTag tag) {
    this.trigger = trigger;
    this.tag = tag;
  }

  /**
   * Compare two events first by tags and, if tags are equal, by trigger names in lexical order.
   * This is useful for enforcing a unique order of events in a priority queue of Event instances.
   */
  @Override
  public int compareTo(Event e) {
    // Compare tags first.
    int ret = this.tag.compareTo(e.getTag());
    // If tags match, compare trigger names.
    if (ret == 0) ret = this.trigger.getFullName().compareTo(e.trigger.getFullName());
    return ret;
  }

  /** This method checks if two events have the same triggers. */
  public boolean hasSameTriggers(Object o) {
    if (o == null) return false;
    if (o instanceof Event) {
      Event e = (Event) o;
      if (this.trigger.equals(e.trigger)) return true;
    }
    return false;
  }

  @Override
  public String toString() {
    return "(" + trigger.getFullName() + ", " + tag + ")";
  }

  public TimeTag getTag() {
    return tag;
  }

  public TriggerInstance<?> getTrigger() {
    return trigger;
  }
}
