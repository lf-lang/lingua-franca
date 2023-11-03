package org.lflang.target.property;

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
}
