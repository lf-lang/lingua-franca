package org.lflang.target.property;

/** Directive to specify that all code is generated in a single file. */
public final class SingleFileProjectProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final SingleFileProjectProperty INSTANCE = new SingleFileProjectProperty();

  private SingleFileProjectProperty() {
    super();
  }

  @Override
  public String name() {
    return "single-file-project";
  }
}
