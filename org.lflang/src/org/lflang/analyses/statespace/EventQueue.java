/**
 * An event queue implementation that
 * sorts events by time tag order
 *
 * 
 */
package org.lflang.analyses.statespace;

import java.util.PriorityQueue;

public class EventQueue extends PriorityQueue<Event> {

    /**
     * Modify the original add() by enforcing uniqueness.
     * There cannot be duplicate events in the event queue.
     */
    @Override
    public boolean add(Event e) {
        if (this.contains(e))
            return false;
        super.add(e);
        return true;
    }
}