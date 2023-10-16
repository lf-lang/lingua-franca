package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive for specifying a specific version of the reactor runtime library. */
public class RuntimeVersionProperty extends StringProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }

  @Override
  public String name() {
    return "runtime-version";
  }
}
