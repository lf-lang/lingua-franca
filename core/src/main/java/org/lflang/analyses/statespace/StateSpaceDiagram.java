package org.lflang.analyses.statespace;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.graph.DirectedGraph;

/** A directed graph representing the state space of an LF program. */
public class StateSpaceDiagram extends DirectedGraph<StateSpaceNode> {

  /** The first node of the state space diagram. */
  public StateSpaceNode head;

  /** The last node of the state space diagram. */
  public StateSpaceNode tail;

  /**
   * The previously encountered node which the tail node goes back to, i.e. the location where the
   * back loop happens.
   */
  public StateSpaceNode loopNode;

  /**
   * Store the state when the loop node is reached for the 2nd time. This is used to calculate the
   * elapsed logical time on the back loop edge.
   */
  public StateSpaceNode loopNodeNext;

  /** The logical time elapsed for each loop iteration. */
  public long loopPeriod;

  /** A dot file that represents the diagram */
  private CodeBuilder dot;

  /** */
  private final boolean compactDot = false;

  /** Before adding the node, assign it an index. */
  @Override
  public void addNode(StateSpaceNode node) {
    node.setIndex(this.nodeCount());
    super.addNode(node);
  }

  /** Get the immediately downstream node. */
  public StateSpaceNode getDownstreamNode(StateSpaceNode node) {
    Set<StateSpaceNode> downstream = this.getDownstreamAdjacentNodes(node);
    if (downstream == null || downstream.size() == 0) return null;
    return (StateSpaceNode) downstream.toArray()[0];
  }

  /** Pretty print the diagram. */
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
    while (node != this.tail) {
      System.out.print("* State " + node.getIndex() + ": ");
      node.display();

      // Store the tag of the prior step.
      timestamp = node.getTag().timestamp;

      // Assume a unique next state.
      node = getDownstreamNode(node);

      // Compute time difference
      if (node != null) {
        TimeValue tsDiff = TimeValue.fromNanoSeconds(node.getTag().timestamp - timestamp);
        System.out.println("*     => Advance time by " + tsDiff);
      }
    }

    // Print tail node
    System.out.print("* (Tail) state " + node.getIndex() + ": ");
    node.display();

    if (this.loopNode != null) {
      // Compute time difference
      TimeValue tsDiff =
          TimeValue.fromNanoSeconds(loopNodeNext.getTag().timestamp - tail.getTag().timestamp);
      System.out.println("*     => Advance time by " + tsDiff);

      System.out.println("* Goes back to loop node: state " + this.loopNode.getIndex());
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
          dot.pr(
              "S"
                  + n.getIndex()
                  + " ["
                  + "label = \" {"
                  + "S"
                  + n.getIndex()
                  + " | "
                  + n.getReactionsInvoked().size()
                  + " | "
                  + n.getEventQcopy().size()
                  + "}"
                  + " | "
                  + n.getTag()
                  + "\""
                  + "]");
        }
      } else {
        for (StateSpaceNode n : this.nodes()) {
          List<String> reactions =
              n.getReactionsInvoked().stream()
                  .map(ReactionInstance::getFullName)
                  .collect(Collectors.toList());
          String reactionsStr = String.join("\\n", reactions);
          List<String> events =
              n.getEventQcopy().stream().map(Event::toString).collect(Collectors.toList());
          String eventsStr = String.join("\\n", events);
          dot.pr(
              "S"
                  + n.getIndex()
                  + " ["
                  + "label = \""
                  + "S"
                  + n.getIndex()
                  + " | "
                  + n.getTag()
                  + " | "
                  + "Reactions invoked:\\n"
                  + reactionsStr
                  + " | "
                  + "Pending events:\\n"
                  + eventsStr
                  + "\""
                  + "]");
        }
      }

      StateSpaceNode current = this.head;
      StateSpaceNode next = getDownstreamNode(this.head);
      while (current != null && next != null && current != this.tail) {
        TimeValue tsDiff =
            TimeValue.fromNanoSeconds(next.getTag().timestamp - current.getTag().timestamp);
        dot.pr(
            "S"
                + current.getIndex()
                + " -> "
                + "S"
                + next.getIndex()
                + " [label = "
                + "\""
                + "+"
                + tsDiff
                + "\""
                + "]");
        current = next;
        next = getDownstreamNode(next);
      }

      if (loopNode != null) {
        TimeValue tsDiff =
            TimeValue.fromNanoSeconds(loopNodeNext.getTag().timestamp - tail.getTag().timestamp);
        TimeValue period = TimeValue.fromNanoSeconds(loopPeriod);
        dot.pr(
            "S"
                + current.getIndex()
                + " -> "
                + "S"
                + next.getIndex()
                + " [label = "
                + "\""
                + "+"
                + tsDiff
                + " -"
                + period
                + "\""
                + " weight = 0 "
                + "]");
      }

      dot.unindent();
      dot.pr("}");
    }
    return this.dot;
  }
}
