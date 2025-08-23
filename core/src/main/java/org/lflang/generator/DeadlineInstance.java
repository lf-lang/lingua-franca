package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Deadline;

/**
 * Instance of a deadline. Upon creation the actual delay is converted into a proper time value. If
 * a parameter is referenced, it is looked up in the given (grand)parent reactor instance.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @ingroup Instances
 */
public class DeadlineInstance {

  /** Create a new deadline instance associated with the given reaction instance. */
  public DeadlineInstance(Deadline definition, ReactionInstance reaction) {
    if (definition.getDelay() != null) {
      this.maxDelay = reaction.parent.getTimeValue(definition.getDelay());
    } else {
      this.maxDelay = TimeValue.ZERO;
    }
  }

  //////////////////////////////////////////////////////
  //// Public fields.

  /**
   * The delay D associated with this deadline. If physical time T < logical time t + D, the
   * deadline is met, otherwise, it is violated.
   */
  public final TimeValue maxDelay;

  //////////////////////////////////////////////////////
  //// Public methods.

  @Override
  public String toString() {
    return "DeadlineInstance " + maxDelay.toString();
  }
}
