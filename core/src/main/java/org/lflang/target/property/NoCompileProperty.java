package org.lflang.target.property;

import org.lflang.generator.GeneratorArguments;

/** If true, do not invoke the target compiler or build command. The default is false. */
public final class NoCompileProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final NoCompileProperty INSTANCE = new NoCompileProperty();

  private NoCompileProperty() {
    super();
  }

  @Override
  public String name() {
    return "no-compile";
  }

  @Override
  public Boolean value(GeneratorArguments args) {
    return args.noCompile;
  }
}
