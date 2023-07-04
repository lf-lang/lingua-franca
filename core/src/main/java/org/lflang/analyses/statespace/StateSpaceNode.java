package org.lflang.analyses.statespace;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.TriggerInstance;

/** A node in the state space diagram representing a step in the execution of an LF program. */
public class StateSpaceNode {

  private int index; // Set in StateSpaceDiagram.java
  private Tag tag;
  private TimeValue time; // Readable representation of tag.timestamp
  private Set<ReactionInstance> reactionsInvoked;
  private ArrayList<Event> eventQcopy; // A snapshot of the eventQ represented as an ArrayList

  public StateSpaceNode(
      Tag tag, Set<ReactionInstance> reactionsInvoked, ArrayList<Event> eventQcopy) {
    this.tag = tag;
    this.eventQcopy = eventQcopy;
    this.reactionsInvoked = reactionsInvoked;
    this.time = TimeValue.fromNanoSeconds(tag.timestamp);
  }

  /**
   * Check if two state space nodes have the same time distance from their respective future events.
   * Given eventQs from both nodes have the same length, check if the time distances between the two
   * nodes' tags and the tags of a pair of events are equal, for all pairs of events (one from n1's
   * eventQ and the other from n2's eventQ),.
   */
  private boolean equidistantNodes(StateSpaceNode n1, StateSpaceNode n2) {
    if (n1.eventQcopy.size() != n2.eventQcopy.size()) return false;
    for (int i = 0; i < n1.eventQcopy.size(); i++) {
      if (n1.eventQcopy.get(i).getTag().timestamp - n1.getTag().timestamp
          != n2.eventQcopy.get(i).getTag().timestamp - n2.getTag().timestamp) {
        return false;
      }
    }
    return true;
  }

  /**
   * Check if two event queues are analogous, meaning that 1) the two event queues have the same
   * size, and 2) each pair of events has the same triggers.
   */
  private boolean analogousEventQs(ArrayList<Event> q1, ArrayList<Event> q2) {
    if (q1.size() != q2.size()) return false;
    for (int i = 0; i < q1.size(); i++) {
      if (!q1.get(i).hasSameTriggers(q2.get(i))) return false;
    }
    return true;
  }

  /** Two methods for pretty printing */
  public void display() {
    System.out.println("(" + this.time + ", " + reactionsInvoked + ", " + eventQcopy + ")");
  }

  public String toString() {
    return "(" + this.time + ", " + reactionsInvoked + ", " + eventQcopy + ")";
  }

  /**
   * This equals method does NOT compare tags, only compares reactionsInvoked, eventQcopy, and
   * whether future events are equally distant.
   *
   * <p>FIXME: Where is this used? This is not used!!!
   */
  @Override
  public boolean equals(Object o) {
    try {
      throw new IOException();
    } catch (IOException e) {
      e.printStackTrace();
    }
    if (o == null) return false;
    if (o instanceof StateSpaceNode) {
      StateSpaceNode node = (StateSpaceNode) o;
      if (this.reactionsInvoked.equals(node.reactionsInvoked)
          && analogousEventQs(this.eventQcopy, node.eventQcopy)
          && equidistantNodes(this, node)) return true;
    }
    return false;
  }

  /**
   * Generate hash for the node. This hash function is used for checking the uniqueness of nodes. It
   * is not meant to be used as a hashCode() because doing so interferes with node insertion in the
   * state space diagram.
   */
  public int hash() {
    // Initial value
    int result = 17;

    // Generate hash for the reactions invoked.
    result = 31 * result + reactionsInvoked.hashCode();

    // Generate hash for the triggers in the queued events.
    List<String> eventNames =
        this.eventQcopy.stream()
            .map(Event::getTrigger)
            .map(TriggerInstance::getFullName)
            .collect(Collectors.toList());
    result = 31 * result + eventNames.hashCode();

    // Generate hash for the time differences.
    List<Long> timeDiff =
        this.eventQcopy.stream()
            .map(
                e -> {
                  return e.getTag().timestamp - this.tag.timestamp;
                })
            .collect(Collectors.toList());
    result = 31 * result + timeDiff.hashCode();

    return result;
  }

  public int getIndex() {
    return index;
  }

  public void setIndex(int i) {
    index = i;
  }

  public Tag getTag() {
    return tag;
  }

  public TimeValue getTime() {
    return time;
  }

  public Set<ReactionInstance> getReactionsInvoked() {
    return reactionsInvoked;
  }

  public ArrayList<Event> getEventQcopy() {
    return eventQcopy;
  }

  public void setEventQcopy(ArrayList<Event> list) {
    eventQcopy = list;
  }
}
