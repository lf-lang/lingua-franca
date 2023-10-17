package org.lflang.target.property;

/**
 * Directive for specifying a path to an external runtime libray to link to instead of the default
 * one.
 */
public final class ExternalRuntimePathProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final ExternalRuntimePathProperty INSTANCE = new ExternalRuntimePathProperty();

  private ExternalRuntimePathProperty() {
    super();
  }

  @Override
  public String name() {
    return "external-runtime-path";
  }
}
