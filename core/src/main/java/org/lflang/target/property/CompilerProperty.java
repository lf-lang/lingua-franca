package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to specify the target compiler. */
public class CompilerProperty extends AbstractStringConfig {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.CPP);
  }

  @Override
  public String name() {
    return "compiler";
  }
}
