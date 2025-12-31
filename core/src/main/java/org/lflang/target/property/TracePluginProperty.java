package org.lflang.target.property;

/** Property that enables an alternative tracing implementation. */
public final class TracePluginProperty extends BooleanProperty {

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
