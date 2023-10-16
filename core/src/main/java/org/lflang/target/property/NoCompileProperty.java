package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/** If true, do not invoke the target compiler or build command. The default is false. */
public class NoCompileProperty extends BooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CPP, Target.CCPP, Target.Python);
  }

  @Override
  public String name() {
    return "no-compile";
  }
}
