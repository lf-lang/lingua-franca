/** 
 * A directed graph representing the state space of an LF program.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.sim;

import java.util.Set;

import org.lflang.graph.DirectedGraph;

// FIXME: Use a linkedlist instead.
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
     * The logical time elapsed for each loop iteration.
     */
    public long loopPeriod;

    /**
     * The length of the state space diagram (not counting the loop)
     */
    public int length;

    /**
     * Before adding the node, assign it an index.
     */
    @Override
    public void addNode(StateSpaceNode node) {
        node.index = this.length;
        this.length++;
        super.addNode(node);
    }

    /**
     * Pretty print the diagram.
     */
    public void display() {
        System.out.println("*************************************************");
        System.out.println("* Pretty printing worst-case state space diagram:");
        StateSpaceNode node = this.head;
        while (node != null) {
            System.out.print("* ");
            System.out.print("State " + node.index + ": ");
            node.display();
            if (!node.equals(this.tail)) {
                // Assume a unique next state.
                Set<StateSpaceNode> downstream = this.getDownstreamAdjacentNodes(node);
                if (downstream == null || downstream.size() == 0) break;
                node = (StateSpaceNode)downstream.toArray()[0];
            }
            else break;
        }
        System.out.println("*************************************************");
    }

}