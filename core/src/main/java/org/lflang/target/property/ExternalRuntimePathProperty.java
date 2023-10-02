package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

public class ExternalRuntimePathProperty extends AbstractStringConfig {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }
}
