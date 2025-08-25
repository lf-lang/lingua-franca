package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Timer;

/**
 * Instance of a timer.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @ingroup Instances
 */
public class TimerInstance extends TriggerInstance<Timer> {

  /** The global default for offset. */
  public static final TimeValue DEFAULT_OFFSET = TimeValue.ZERO;

  /** The global default for period. */
  public static final TimeValue DEFAULT_PERIOD = TimeValue.ZERO;

  private TimeValue offset = DEFAULT_OFFSET;

  private TimeValue period = DEFAULT_PERIOD;

  /**
   * Create a new timer instance. If the definition is null, then this is a startup timer.
   *
   * @param definition The AST definition, or null for startup.
   * @param parent The parent reactor.
   */
  public TimerInstance(Timer definition, ReactorInstance parent) {
    super(definition, parent);
    if (parent == null) {
      throw new InvalidSourceException("Cannot create an TimerInstance with no parent.");
    }
    if (definition != null) {
      if (definition.getOffset() != null) {
        try {
          this.offset = parent.getTimeValue(definition.getOffset());
        } catch (IllegalArgumentException ex) {
          parent.reporter.at(definition.getOffset()).error("Invalid time.");
        }
      }
      if (definition.getPeriod() != null) {
        try {
          this.period = parent.getTimeValue(definition.getPeriod());
        } catch (IllegalArgumentException ex) {
          parent.reporter.at(definition.getPeriod()).error("Invalid time.");
        }
      }
    }
  }

  /** Get the value of the offset parameter. */
  public TimeValue getOffset() {
    return offset;
  }

  /** Get the value of the offset parameter. */
  public TimeValue getPeriod() {
    return period;
  }
}
