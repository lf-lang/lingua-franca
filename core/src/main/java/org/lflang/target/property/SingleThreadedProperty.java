package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;

/** Directive to indicate whether the runtime should use multi-threading. */
public class SingleThreadedProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final SingleThreadedProperty INSTANCE = new SingleThreadedProperty();

  private SingleThreadedProperty() {
    super();
  }

  @Override
  public String name() {
    return "single-threaded";
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    if (config.isFederated() && config.get(this).equals(true)) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
          .error("Cannot enable single-threaded mode for federated program.");
    }
  }
}
