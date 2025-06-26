package org.lflang.analyses.statespace;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.graph.DirectedGraph;
import org.lflang.pretvm.ExecutionPhase;

/**
 * A directed graph representing the state space of an LF program.
 *
 * @author Shaokai J. Lin
 */
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

  /**
   * The logical time elapsed for each loop iteration. With the assumption of "logical time =
   * physical time," this is also the hyperperiod in physical time.
   */
  public long hyperperiod;

  /** The exploration phase in which this diagram is generated */
  public ExecutionPhase phase;

  /** A dot file that represents the diagram */
  private CodeBuilder dot;

  /** A flag that indicates whether we want the dot to be compact */
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
    TimeValue time;
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
      time = node.getTag().time;

      // Assume a unique next state.
      node = getDownstreamNode(node);

      // Compute time difference
      if (node != null) {
        TimeValue tsDiff =
            TimeValue.fromNanoSeconds(node.getTag().time.toNanoSeconds() - time.toNanoSeconds());
        System.out.println("*     => Advance time by " + tsDiff);
      }
    }

    // Print tail node
    System.out.print("* (Tail) state " + node.getIndex() + ": ");
    node.display();

    if (this.loopNode != null) {
      // Compute time difference
      TimeValue tsDiff =
          TimeValue.fromNanoSeconds(
              loopNodeNext.getTag().time.toNanoSeconds() - tail.getTag().time.toNanoSeconds());
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
      if (this.isCyclic()) {
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
            TimeValue.fromNanoSeconds(
                next.getTag().time.toNanoSeconds() - current.getTag().time.toNanoSeconds());
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
            TimeValue.fromNanoSeconds(
                loopNodeNext.getTag().time.toNanoSeconds() - tail.getTag().time.toNanoSeconds());
        TimeValue period = TimeValue.fromNanoSeconds(hyperperiod);
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

  public void generateDotFile(Path dir, String filename) {
    try {
      Path path = dir.resolve(filename);
      CodeBuilder dot = generateDot();
      dot.writeToFile(path.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** Check if the diagram is periodic by checking if the loop node is set. */
  public boolean isCyclic() {
    return loopNode != null;
  }

  /** Check if the diagram is empty. */
  public boolean isEmpty() {
    return (head == null);
  }
}
