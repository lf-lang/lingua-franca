package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/** Flags to be passed on to the target compiler. */
public class CompilerFlagsProperty extends AbstractStringListProperty {

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public String name() {
    return "compiler-flags";
  }
}
