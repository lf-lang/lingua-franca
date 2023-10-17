package org.lflang.target.property;

/** If true, do not perform runtime validation. The default is false. */
public final class NoRuntimeValidationProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final NoRuntimeValidationProperty INSTANCE = new NoRuntimeValidationProperty();

  private NoRuntimeValidationProperty() {
    super();
  }

  @Override
  public String name() {
    return "no-runtime-validation";
  }
}
