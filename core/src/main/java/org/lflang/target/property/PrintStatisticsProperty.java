package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to instruct the runtime to collect and print execution statistics. */
public class PrintStatisticsProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "print-statistics";
  }
}
