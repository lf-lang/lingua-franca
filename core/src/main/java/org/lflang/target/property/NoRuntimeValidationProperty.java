package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to disable validation of reactor rules at runtime. */
public class NoRuntimeValidationProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "no-runtime-validation";
  }
}
