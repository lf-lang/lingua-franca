package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** If true, instruct the runtime to collect and print execution statistics. */
public final class PrintStatisticsProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final PrintStatisticsProperty INSTANCE = new PrintStatisticsProperty();

  private PrintStatisticsProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "print-statistics";
  }
}
