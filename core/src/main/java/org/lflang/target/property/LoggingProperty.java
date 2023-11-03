package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.LoggingType;
import org.lflang.target.property.type.LoggingType.LogLevel;

/**
 * Directive to specify the grain at which to report log messages during execution. The default is
 * INFO.
 */
public final class LoggingProperty extends TargetProperty<LogLevel, LoggingType> {

  /** Singleton target property instance. */
  public static final LoggingProperty INSTANCE = new LoggingProperty();

  private LoggingProperty() {
    super(new LoggingType());
  }

  @Override
  public LogLevel initialValue() {
    return LogLevel.getDefault();
  }

  @Override
  protected LogLevel fromAst(Element node, MessageReporter reporter) {
    return fromString(ASTUtils.elementToSingleString(node), reporter);
  }

  protected LogLevel fromString(String string, MessageReporter reporter) {
    return LogLevel.valueOf(string.toUpperCase());
  }

  @Override
  public Element toAstElement(LogLevel value) {
    return ASTUtils.toElement(value.toString());
  }

  @Override
  public String name() {
    return "logging";
  }
}
