package org.lflang.target.property;

import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.LoggingProperty.LogLevel;
import org.lflang.target.property.type.UnionType;

public class LoggingProperty extends AbstractTargetProperty<LogLevel> {

  public LoggingProperty() {
    super(UnionType.LOGGING_UNION);
  }

  @Override
  public LogLevel initialValue() {
    return LogLevel.INFO;
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
  }
}
