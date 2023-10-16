package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** The compiler to invoke, unless a build command has been specified. */
public class CompilerProperty extends StringProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.CPP);
  }

  @Override
  public String name() {
    return "compiler";
  }
}
