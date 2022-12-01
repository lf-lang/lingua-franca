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

    public Tag tag;
    public ArrayList<ReactionInstance> reactions_invoked;
    public ArrayList<Event> eventQ;

    public StateSpaceNode(
        Tag tag,
        ArrayList<ReactionInstance> reactions_invoked,
        ArrayList<Event> eventQ
    ) {
        this.tag = tag;
        this.reactions_invoked = reactions_invoked;
        this.eventQ = eventQ;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o instanceof StateSpaceNode) {
            StateSpaceNode node = (StateSpaceNode) o;
            if (this.tag.equals(node.tag)
                && this.reactions_invoked.equals(node.reactions_invoked)
                && this.eventQ.equals(node.eventQ))
                return true;
        }
        return false;
    }

    public void display() {
        System.out.println("(" + tag + ", " + reactions_invoked + ", " + eventQ + ")");
    }
    
}