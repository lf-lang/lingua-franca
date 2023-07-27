package org.lflang.analyses.statespace;

import java.util.PriorityQueue;

/**
 * An event queue implementation that sorts events in the order of _time tags_ and _trigger names_
 * based on the implementation of compareTo() in the Event class.
 */
public class EventQueue extends PriorityQueue<Event> {

  /**
   * Modify the original add() by enforcing uniqueness. There cannot be duplicate events in the
   * event queue.
   */
  @Override
  public boolean add(Event e) {
    if (this.contains(e)) return false;
    super.add(e);
    return true;
  }
}
