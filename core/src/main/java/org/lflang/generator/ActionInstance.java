package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;

/**
 * Instance of an action.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @ingroup Instances
 */
public class ActionInstance extends TriggerInstance<Action> {

  /** The constant default for a minimum delay. */
  public static final TimeValue DEFAULT_MIN_DELAY = TimeValue.ZERO;

  /** The minimum delay for this action. */
  private TimeValue minDelay = DEFAULT_MIN_DELAY;

  /** The minimum spacing between action events. */
  private TimeValue minSpacing = null;

  /** The replacement policy for when minimum spacing is violated. */
  private String policy = null;

  /** Indicator of whether the action is physical. */
  private boolean physical;

  /**
   * Create a new action instance. If the definition is null, then this is a shutdown action.
   *
   * @param definition The AST definition, or null for startup.
   * @param parent The parent reactor.
   */
  public ActionInstance(Action definition, ReactorInstance parent) {
    super(definition, parent);
    if (parent == null) {
      throw new InvalidSourceException("Cannot create an ActionInstance with no parent.");
    }
    if (definition != null) {
      if (definition.getMinDelay() != null) {
        this.minDelay = parent.getTimeValue(definition.getMinDelay());
      }
      if (definition.getMinSpacing() != null) {
        this.minSpacing = parent.getTimeValue(definition.getMinSpacing());
      }
      if (definition.getOrigin() == ActionOrigin.PHYSICAL) {
        physical = true;
      }
      policy = definition.getPolicy();
    }
  }

  /** Return the minimum delay for this action. */
  public TimeValue getMinDelay() {
    return minDelay;
  }

  /** Return the minimum spacing between action events. */
  public TimeValue getMinSpacing() {
    return minSpacing;
  }

  /** Return the replacement policy for when minimum spacing is violated. */
  public String getPolicy() {
    return policy;
  }

  /** Return the indicator of whether the action is physical. */
  public boolean isPhysical() {
    return physical;
  }
}
