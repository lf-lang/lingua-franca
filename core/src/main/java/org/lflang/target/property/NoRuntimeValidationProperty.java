package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** If true, do not perform runtime validation. The default is false. */
public class NoRuntimeValidationProperty extends BooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "no-runtime-validation";
  }
}
