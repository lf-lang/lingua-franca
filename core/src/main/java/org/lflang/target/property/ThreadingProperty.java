package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to indicate whether the runtime should use multi-threading. */
public class ThreadingProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python, Target.Rust);
  }

  @Override
  public String name() {
    return "threading";
  }

  @Override
  public Boolean initialValue() {
    return true;
  }
}
