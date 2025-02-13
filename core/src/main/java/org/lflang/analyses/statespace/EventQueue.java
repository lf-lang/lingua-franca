package org.lflang.analyses.statespace;

import java.util.PriorityQueue;

/** An event queue for analyzing the logical behavior of an LF program */
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
