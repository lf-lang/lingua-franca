/** 
 * A node in the state space diagram representing a step
 * in the execution of an LF program.
 * 
 * 
 */
package org.lflang.analyses.statespace;

import org.lflang.generator.TriggerInstance;

public class Event implements Comparable<Event> {

    public TriggerInstance trigger;
    public Tag tag;

    public Event(TriggerInstance trigger, Tag tag) {
        this.trigger = trigger;
        this.tag = tag;
    }

    public TriggerInstance getTrigger() {
        return this.trigger;
    }
    
    @Override
    public int compareTo(Event e) {
        // Compare tags first.
        int ret = this.tag.compareTo(e.tag);
        // If tags match, compare trigger names.
        if (ret == 0)
            ret = this.trigger.getFullName()
                              .compareTo(e.trigger.getFullName());
        return ret;
    }

    /**
     * This equals() method does NOT compare tags,
     * only compares triggers.
     */
    @Override
    public boolean equals(Object o) {
        if (o == null) return false;
        if (o instanceof Event) {
            Event e = (Event) o;
            if (this.trigger.equals(e.trigger))
                return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "(" + trigger.getFullName() + ", " + tag + ")";
    }
}