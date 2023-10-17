package org.lflang.target.property;

/** Directive for specifying a specific version of the reactor runtime library. */
public final class RuntimeVersionProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final RuntimeVersionProperty INSTANCE = new RuntimeVersionProperty();

  private RuntimeVersionProperty() {
    super();
  }

  @Override
  public String name() {
    return "runtime-version";
  }
}
