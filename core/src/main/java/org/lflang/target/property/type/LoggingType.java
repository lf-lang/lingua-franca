package org.lflang.target.property.type;

import org.lflang.target.property.type.LoggingType.LogLevel;

public class LoggingType extends OptionsType<LogLevel> {

  @Override
  protected Class<LogLevel> enumClass() {
    return LogLevel.class;
  }

  /**
   * Log levels in descending order of severity.
   *
   * @author Marten Lohstroh
   */
  public enum LogLevel {
    ERROR,
    WARN,
    INFO,
    LOG,
    DEBUG;

    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }

    public static LogLevel getDefault() {
      return LogLevel.INFO;
    }
  }
}
