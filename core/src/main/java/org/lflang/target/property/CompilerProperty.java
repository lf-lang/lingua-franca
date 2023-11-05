package org.lflang.target.property;

/** The compiler to invoke, unless a build command has been specified. */
public final class CompilerProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final CompilerProperty INSTANCE = new CompilerProperty();

  private CompilerProperty() {
    super();
  }

  @Override
  public String name() {
    return "compiler";
  }
}
