package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/** Flags to pass to the compiler, unless a build command has been specified. */
public final class CompilerFlagsProperty extends StringListProperty {

  /** Singleton target property instance. */
  public static final CompilerFlagsProperty INSTANCE = new CompilerFlagsProperty();

  private CompilerFlagsProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public String name() {
    return "compiler-flags";
  }
}
