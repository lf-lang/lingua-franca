package org.lflang.target.property;

/** Directive to indicate whether the runtime should use multi-threading. */
public class SingleThreadedProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final SingleThreadedProperty INSTANCE = new SingleThreadedProperty();

  private SingleThreadedProperty() {
    super();
  }

  @Override
  public String name() {
    return "single-threaded";
  }

  @Override
  public Boolean initialValue() {
    return true;
  }
}
