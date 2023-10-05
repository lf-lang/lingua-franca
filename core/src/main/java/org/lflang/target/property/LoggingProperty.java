package org.lflang.target.property;

import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.LoggingType;
import org.lflang.target.property.type.LoggingType.LogLevel;

/**
 * Directive to specify the grain at which to report log messages during execution. The default is
 * INFO.
 */
public class LoggingProperty extends AbstractTargetProperty<LogLevel, LoggingType> {

  public LoggingProperty() {
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
  public List<Target> supportedTargets() {
    return Target.ALL;
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(get().toString());
  }

  @Override
  public String name() {
    return "logging";
  }
}
