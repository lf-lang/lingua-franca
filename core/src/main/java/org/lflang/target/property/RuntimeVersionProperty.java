package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

public class RuntimeVersionProperty extends AbstractStringConfig {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }

  @Override
  public String name() {
    return "runtime-version";
  }
}
