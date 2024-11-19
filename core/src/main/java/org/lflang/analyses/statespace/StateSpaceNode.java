package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.TriggerInstance;

/**
 * A node in the state space diagram representing a step in the execution of an LF program.
 *
 * 
 */
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

  /** Copy constructor */
  public StateSpaceNode(StateSpaceNode that) {
    this.tag = new Tag(that.tag);
    this.eventQcopy = new ArrayList<>(that.eventQcopy);
    this.reactionsInvoked = new HashSet<>(that.reactionsInvoked);
    this.time = TimeValue.fromNanoSeconds(that.tag.timestamp);
  }

  /** Two methods for pretty printing */
  public void display() {
    System.out.println("(" + this.time + ", " + reactionsInvoked + ", " + eventQcopy + ")");
  }

  public String toString() {
    return "(" + this.time + ", " + reactionsInvoked + ", " + eventQcopy + ")";
  }

  /**
   * Generate hash for the node. This hash function is used for checking whether two nodes are
   * analogous, meaning that 1) they have the same reactions invoked at their tags, 2) the queued
   * events have the same triggers, and 3) the time offsets between future events' tags of one node
   * and its tag are the same as the time offsets between future events' tags of the other node and
   * the other node's tag. The hash() method is not meant to replace the hashCode() method because
   * doing so changes the way nodes are inserted in the state space diagram.
   */
  public int hash() {
    // Initial value
    int result = 17;

    // Generate hash for the reactions invoked.
    result = 31 * result + reactionsInvoked.hashCode();

    // Generate hash for the triggers in the queued events.
    int eventsHash =
        this.getEventQcopy().stream()
            .map(Event::getTrigger)
            .map(TriggerInstance::getFullName)
            .mapToInt(Object::hashCode)
            .reduce(1, (a, b) -> 31 * a + b);
    result = 31 * result + eventsHash;

    // Generate hash for the time differences.
    long timeDiffHash =
        this.getEventQcopy().stream()
            .mapToLong(e -> e.getTag().timestamp - this.tag.timestamp)
            .reduce(1, (a, b) -> 31 * a + b);
    result = 31 * result + (int) timeDiffHash;

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

  public void setTag(Tag newTag) {
    tag = newTag;
    time = TimeValue.fromNanoSeconds(tag.timestamp);
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
