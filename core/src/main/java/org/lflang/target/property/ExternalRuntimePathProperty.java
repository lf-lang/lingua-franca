package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive for specifying a path to an external runtime libray to link to instead of the default
 * one.
 */
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
