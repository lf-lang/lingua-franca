/** 
 * A directed graph representing the state space of an LF program.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
package org.lflang.analyses.statespace;

import java.util.List;
import java.util.Set;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
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
     * 
     */
    private final boolean compactDot = false;

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
        long timestamp;
        StateSpaceNode node = this.head;
        if (node == null) {
            System.out.println("* EMPTY");
            System.out.println("*************************************************");
            return;
        }
        while(node != this.tail) {
            System.out.print("* State " + node.index + ": ");
            node.display();

            // Store the tag of the prior step.
            timestamp = node.tag.timestamp;

            // Assume a unique next state.
            node = getDownstreamNode(node);

            // Compute time difference
            if (node != null) {
                TimeValue tsDiff = TimeValue.fromNanoSeconds(node.tag.timestamp - timestamp);
                System.out.println("*     => Advance time by " + tsDiff);
            }
        }

        // Print tail node
        System.out.print("* (Tail) state " + node.index + ": ");
        node.display();

        if (this.loopNode != null) {
            // Compute time difference
            TimeValue tsDiff = TimeValue.fromNanoSeconds(loopNodeNext.tag.timestamp - tail.tag.timestamp);
            System.out.println("*     => Advance time by " + tsDiff);

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
            if (this.loopNode != null) {
                dot.pr("layout=circo;");
            }
            dot.pr("rankdir=LR;");
            if (this.compactDot) {
                dot.pr("mindist=1.5;");
                dot.pr("overlap=false");
                dot.pr("node [shape=Mrecord fontsize=40]");
                dot.pr("edge [fontsize=40 penwidth=3 arrowsize=2]");
            } else {
                dot.pr("node [shape=Mrecord]");
            }
            // Generate a node for each state.
            if (this.compactDot) {
                for (StateSpaceNode n : this.nodes()) {
                    dot.pr("S" + n.index + " [" + "label = \" {" + "S" + n.index
                        + " | " + n.reactionsInvoked.size() + " | " + n.eventQ.size() + "}"
                        + " | " + n.tag
                        + "\"" + "]");
                }
            } else {
                System.out.println("***** nodes: " + this.nodes());
                for (StateSpaceNode n : this.nodes()) {
                    List<String> reactions = n.reactionsInvoked.stream()
                        .map(ReactionInstance::getFullName).collect(Collectors.toList());
                    String reactionsStr = String.join("\\n", reactions);
                    List<String> events = n.eventQ.stream()
                        .map(Event::toString).collect(Collectors.toList());
                    String eventsStr = String.join("\\n", events);
                    dot.pr("S" + n.index + " [" + "label = \"" + "S" + n.index
                        + " | " + n.tag
                        + " | " + "Reactions invoked:\\n" + reactionsStr
                        + " | " + "Pending events:\\n" + eventsStr
                        + "\"" + "]");
                }
            }

            StateSpaceNode current = this.head;
            StateSpaceNode next = getDownstreamNode(this.head);
            while (current != null && next != null && current != this.tail) {
                TimeValue tsDiff = TimeValue.fromNanoSeconds(next.tag.timestamp - current.tag.timestamp);
                dot.pr("S" + current.index + " -> " + "S" + next.index + " [label = " + "\"" + "+" + tsDiff + "\"" + "]");
                current = next;
                next = getDownstreamNode(next);
            }

            if (loopNode != null) {
                TimeValue tsDiff = TimeValue.fromNanoSeconds(loopNodeNext.tag.timestamp - tail.tag.timestamp);
                TimeValue period = TimeValue.fromNanoSeconds(loopPeriod);
                dot.pr("S" + current.index + " -> " + "S" + next.index
                    + " [label = " + "\"" + "+" + tsDiff + " -" + period + "\""
                    + " weight = 0 " + "]");
            }

            dot.unindent();
            dot.pr("}");
        }
        return this.dot;
    }
}