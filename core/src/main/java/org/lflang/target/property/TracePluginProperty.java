package org.lflang.target.property;

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
