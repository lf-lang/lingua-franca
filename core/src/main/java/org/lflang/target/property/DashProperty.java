package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.SchedulerType.Scheduler;

/**
 * If true, configure the execution environment such that it does not wait for physical time to
 * match logical time for non-real-time reactions. A reaction is real-time if it is
 * within a real-time reactor (marked by the `realtime` keyword). The default is false.
 */
public final class DashProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final DashProperty INSTANCE = new DashProperty();

  private DashProperty() {
    super();
  }

  @Override
  public String name() {
    return "dash";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var pair = config.lookup(this);
    if (config.isSet(this) && config.isFederated()) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .error("The dash target property is incompatible with federated programs.");
    }

    if (!(config.target == Target.C 
      && config.get(SchedulerProperty.INSTANCE).type() == Scheduler.STATIC)) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .error(
              String.format(
                  "The dash mode currently only works in the C target with the STATIC scheduler.",
                  config.target.toString()));      
    }
  }
}
