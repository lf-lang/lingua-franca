/** 
 * A directed graph representing the state space of an LF program.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.sim;

import java.util.Set;

import org.lflang.graph.DirectedGraph;

public class StateSpaceDiagram extends DirectedGraph<StateSpaceNode> {

    /**
     * The first node of the state space diagram.
     */
    public StateSpaceNode head;

    /**
     * The last node of the state space diagram.
     */
    public StateSpaceNode tail;

    /**
     * The previously encountered node which the tail node
     * goes back to, i.e. the location where the back loop happens.
     */
    public StateSpaceNode loopNode;

    /**
     * Pretty print the diagram.
     */
    public void display() {
        System.out.println("Pretty printing state space diagram:");
        StateSpaceNode node = this.head;
        int count = 0;
        while (node != null) {
            System.out.print("State " + count++ + ": ");
            node.display();
            if (!node.equals(this.tail)) {
                // Assume a unique next state.
                Set<StateSpaceNode> downstream = this.getDownstreamAdjacentNodes(node);
                if (downstream == null || downstream.size() == 0) break;
                node = (StateSpaceNode)downstream.toArray()[0];
            }
        }
    }

}