package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive for specifying a path to an external runtime to be used for the compiled binary. */
public class ExternalRuntimePathProperty extends AbstractStringConfig {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }

  @Override
  public String name() {
    return "external-runtime-path";
  }
}
