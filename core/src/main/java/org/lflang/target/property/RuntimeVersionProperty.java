package org.lflang.target.property;

import org.lflang.generator.GeneratorArguments;

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

  @Override
  public String value(GeneratorArguments args) {
    return args.runtimeVersion;
  }
}
