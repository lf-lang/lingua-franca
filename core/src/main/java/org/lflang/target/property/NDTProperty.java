package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.LoggingType;
import org.lflang.target.property.type.LoggingType.LogLevel;

/**
 * FIXME: Add the NDT property.
 */
public final class NDTProperty extends TargetProperty<LogLevel, LoggingType> {

  /** Singleton target property instance. */
  public static final NDTProperty INSTANCE = new NDTProperty();

  private NDTProperty() {
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
    return "disabled";
  }
}
