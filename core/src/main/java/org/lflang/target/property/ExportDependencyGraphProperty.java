package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/**
 * If true, the resulting binary will output a graph visualizing all reaction dependencies.
 *
 * <p>This option is currently only used for C++ and Rust. This export function is a valuable tool
 * for debugging LF programs and helps to understand the dependencies inferred by the runtime.
 */
public final class ExportDependencyGraphProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final ExportDependencyGraphProperty INSTANCE = new ExportDependencyGraphProperty();

  private ExportDependencyGraphProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP, Target.Rust);
  }

  @Override
  public String name() {
    return "export-dependency-graph";
  }
}
