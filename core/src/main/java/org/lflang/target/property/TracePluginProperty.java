package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.PrimitiveType;

/** Property that provides an alternative tracing implementation. */
/** The compiler to invoke, unless a build command has been specified. */
public final class TracePluginProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final TracePluginProperty INSTANCE = new TracePluginProperty();

  private TracePluginProperty() {
    super();
  }

  @Override
  public String name() {
    return "trace-plugin";
  }
}
