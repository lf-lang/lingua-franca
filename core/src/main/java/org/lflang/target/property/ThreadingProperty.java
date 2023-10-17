package org.lflang.target.property;

/** Directive to indicate whether the runtime should use multi-threading. */
public class ThreadingProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final ThreadingProperty INSTANCE = new ThreadingProperty();

  private ThreadingProperty() {
    super();
  }

  @Override
  public String name() {
    return "threading";
  }

  @Override
  public Boolean initialValue() {
    return true;
  }
}
