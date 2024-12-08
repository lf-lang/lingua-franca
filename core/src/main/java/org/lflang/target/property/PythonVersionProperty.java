package org.lflang.target.property;

/** A specific Python version to use. */
public final class PythonVersionProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final PythonVersionProperty INSTANCE = new PythonVersionProperty();

  private PythonVersionProperty() {
    super();
  }

  @Override
  public String name() {
    return "python-version";
  }
}
