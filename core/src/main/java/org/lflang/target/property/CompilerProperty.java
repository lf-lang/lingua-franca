package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** The compiler to invoke, unless a build command has been specified. */
public final class CompilerProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final CompilerProperty INSTANCE = new CompilerProperty();

  private CompilerProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.CPP);
  }

  @Override
  public String name() {
    return "compiler";
  }
}
