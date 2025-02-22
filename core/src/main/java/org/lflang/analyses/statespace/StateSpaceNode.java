package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import org.lflang.TimeTag;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.TriggerInstance;

/** A node in the state space diagram representing a step in the execution of an LF program. */
public class StateSpaceNode {

  private int index; // An integer ID for this node
  private TimeTag tag;
  private Set<ReactionInstance> reactionsInvoked;
  private ArrayList<Event> eventQcopy; // A snapshot of the eventQ represented as an ArrayList

  public StateSpaceNode(
      TimeTag tag, Set<ReactionInstance> reactionsInvoked, ArrayList<Event> eventQcopy) {
    this.tag = tag;
    this.eventQcopy = eventQcopy;
    this.reactionsInvoked = reactionsInvoked;
  }

  /** Copy constructor */
  public StateSpaceNode(StateSpaceNode that) {
    this.tag = new TimeTag(that.tag);
    this.eventQcopy = new ArrayList<>(that.eventQcopy);
    this.reactionsInvoked = new HashSet<>(that.reactionsInvoked);
  }

  /** Two methods for pretty printing */
  public void display() {
    System.out.println("(" + this.tag.time + ", " + reactionsInvoked + ", " + eventQcopy + ")");
  }

  public String toString() {
    return "(" + this.tag.time + ", " + reactionsInvoked + ", " + eventQcopy + ")";
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
            .mapToLong(e -> e.getTag().time.toNanoSeconds() - this.tag.time.toNanoSeconds())
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

  public TimeTag getTag() {
    return tag;
  }

  public void setTag(TimeTag newTag) {
    tag = newTag;
  }

  public TimeValue getTime() {
    return tag.time;
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
