package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * Directive to let the runtime export its internal dependency graph.
 *
 * <p>This is a debugging feature and currently only used for C++ and Rust programs.
 */
public class ExportDependencyGraphProperty extends AbstractBooleanProperty {

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }

  @Override
  public String name() {
    return "export-dependency-graph";
  }
}
