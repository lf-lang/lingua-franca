/** 
 * A node in the state space diagram representing a step
 * in the execution of an LF program.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.sim;

import java.util.ArrayList;

import org.lflang.generator.ReactionInstance;

public class StateSpaceNode {

    public int index; // Set in StateSpaceDiagram.java
    public Tag tag;
    public ArrayList<ReactionInstance> reactionsInvoked;
    public ArrayList<Event> eventQ;

    public StateSpaceNode(
        Tag tag,
        ArrayList<ReactionInstance> reactionsInvoked,
        ArrayList<Event> eventQ
    ) {
        this.tag    = tag;
        this.eventQ = eventQ;
        this.reactionsInvoked = reactionsInvoked;
    }

    /**
     * Assuming both eventQs have the same length,
     * for each pair of events in eventQ1 and eventQ2,
     * check if the time distances between the node's tag
     * and the two events' tags are equal.
     */
    private boolean equidistant(StateSpaceNode n1,
                                StateSpaceNode n2) {
        if (n1.eventQ.size() != n2.eventQ.size())
            return false;
        for (int i = 0; i < n1.eventQ.size(); i++) {
            if (n1.eventQ.get(i).tag.timestamp - n1.tag.timestamp
                != n2.eventQ.get(i).tag.timestamp - n2.tag.timestamp) {
                return false;
            }
        }
        return true;
    }

    /**
     * This equals method does NOT compare tags,
     * only compares reactionsInvoked and eventQ.
     */
    @Override
    public boolean equals(Object o) {
        if (o == null) return false;
        if (o instanceof StateSpaceNode) {
            StateSpaceNode node = (StateSpaceNode) o;
            if (this.reactionsInvoked.equals(node.reactionsInvoked)
                && this.eventQ.equals(node.eventQ)
                && equidistant(this, node))
                return true;
        }
        return false;
    }

    public void display() {
        System.out.println("(" + tag + ", " + reactionsInvoked + ", " + eventQ + ")");
    }
    
    public String toString() {
        return "(" + tag + ", " + reactionsInvoked + ", " + eventQ + ")";
    }
}