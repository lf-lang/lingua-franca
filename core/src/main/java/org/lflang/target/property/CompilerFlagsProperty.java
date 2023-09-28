package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

public class CompilerFlagsProperty extends DefaultStringListProperty {

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }
}
