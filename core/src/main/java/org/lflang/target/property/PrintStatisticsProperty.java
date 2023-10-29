package org.lflang.target.property;

import org.lflang.generator.GeneratorArguments;

/** If true, instruct the runtime to collect and print execution statistics. */
public final class PrintStatisticsProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final PrintStatisticsProperty INSTANCE = new PrintStatisticsProperty();

  private PrintStatisticsProperty() {
    super();
  }

  @Override
  public String name() {
    return "print-statistics";
  }

  @Override
  public Boolean value(GeneratorArguments args) {
    return args.printStatistics;
  }
}
