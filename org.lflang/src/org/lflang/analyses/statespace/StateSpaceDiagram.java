/** 
 * A directed graph representing the state space of an LF program.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.analyses.statespace;

import java.util.Set;

import org.lflang.generator.CodeBuilder;
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
     * Store the state when the loop node is reached for
     * the 2nd time. This is used to calculate the elapsed
     * logical time on the back loop edge.
     */
    public StateSpaceNode loopNodeNext;

    /**
     * The logical time elapsed for each loop iteration.
     */
    public long loopPeriod;

    /**
     * A dot file that represents the diagram
     */
    private CodeBuilder dot;

    /**
     * Before adding the node, assign it an index.
     */
    @Override
    public void addNode(StateSpaceNode node) {
        node.index = this.nodeCount();
        super.addNode(node);
    }

    /**
     * Get the immediately downstream node.
     */
    public StateSpaceNode getDownstreamNode(StateSpaceNode node) {
        Set<StateSpaceNode> downstream = this.getDownstreamAdjacentNodes(node);
        if (downstream == null || downstream.size() == 0)
            return null;
        return (StateSpaceNode)downstream.toArray()[0];
    }

    /**
     * Pretty print the diagram.
     */
    public void display() {
        System.out.println("*************************************************");
        System.out.println("* Pretty printing worst-case state space diagram:");
        StateSpaceNode node = this.head;
        long timestamp;
        long microstep;
        while (node != null) {
            System.out.print("* State " + node.index + ": ");
            node.display();

            // Store the tag of the prior step.
            timestamp = node.tag.timestamp;
            microstep = node.tag.microstep;

            if (!node.equals(this.tail)) {
                // Assume a unique next state.
                node = getDownstreamNode(node);

                // Compute time difference
                if (node != null) {
                    timestamp = node.tag.timestamp - timestamp;
                    microstep = node.tag.microstep - microstep;
                    System.out.println("*     => Advance time by (" + timestamp + ", " + microstep + ")");
                }
            }
            else break;
        }
        if (this.loopNode != null) {
            // Compute time difference
            timestamp = loopNodeNext.tag.timestamp - tail.tag.timestamp;
            microstep = loopNodeNext.tag.microstep - tail.tag.microstep;
            System.out.println("*     => Advance time by (" + timestamp + ", " + microstep + ")");

            System.out.println("* Goes back to loop node: state " + this.loopNode.index);
            System.out.print("* Loop node reached 2nd time: ");
            this.loopNodeNext.display();
        }
        System.out.println("*************************************************");
    }

    /**
     * Generate a dot file from the state space diagram.
     * 
     * @return a CodeBuilder with the generated code
     */
    public CodeBuilder generateDot() {
        if (dot == null) {
            dot = new CodeBuilder();
            dot.pr("digraph G {");
            dot.indent();
            dot.pr("rankdir=\"LR\";");
            dot.pr("node [shape=Mrecord]");

            // Generate a node for each state.
            for (StateSpaceNode n : this.nodes()) {
                dot.pr("S" + n.index + " [" + "label = \"" + "S" + n.index + " | " + n.tag + "\"" + "]");
            }

            StateSpaceNode current = this.head;
            StateSpaceNode next = getDownstreamNode(this.head);
            while (current != null && next != null && current != this.tail) {
                long tsDiff = next.tag.timestamp - current.tag.timestamp;
                long msDiff = next.tag.microstep - current.tag.microstep;
                dot.pr("S" + current.index + " -> " + "S" + next.index + " [label = " + "\"" + "+(" + tsDiff + ", " + msDiff + ")" + "\"" + "]");
                current = next;
                next = getDownstreamNode(next);
            }

            if (loopNode != null) {
                long tsDiff = loopNodeNext.tag.timestamp - tail.tag.timestamp;
                long msDiff = loopNodeNext.tag.microstep - tail.tag.microstep;
                dot.pr("S" + current.index + " -> " + "S" + next.index
                    + " [label = " + "\"" + "+(" + tsDiff + ", " + msDiff + ")" + "\"" + " weight = 0 " + "]");
            }

            dot.unindent();
            dot.pr("}");
        }
        return this.dot;
    }
}