package org.lflang.target.property;

/** Flags to pass to the compiler, unless a build command has been specified. */
public final class CompilerFlagsProperty extends StringListProperty {

  /** Singleton target property instance. */
  public static final CompilerFlagsProperty INSTANCE = new CompilerFlagsProperty();

  private CompilerFlagsProperty() {
    super();
  }

  @Override
  public String name() {
    return "compiler-flags";
  }
}
