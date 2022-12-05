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

    public int index;
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
     * This equals method does NOT compare tags,
     * only compares reactionsInvoked and eventQ.
     */
    @Override
    public boolean equals(Object o) {
        if (o == null) return false;
        if (o instanceof StateSpaceNode) {
            StateSpaceNode node = (StateSpaceNode) o;
            if (this.reactionsInvoked.equals(node.reactionsInvoked)
                && this.eventQ.equals(node.eventQ))
                return true;
        }
        return false;
    }

    public void display() {
        System.out.println("(" + tag + ", " + reactionsInvoked + ", " + eventQ + ")");
    }
    
}