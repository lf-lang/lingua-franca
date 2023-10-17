package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** Directive to specify that all code is generated in a single file. */
public final class SingleFileProjectProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final SingleFileProjectProperty INSTANCE = new SingleFileProjectProperty();

  private SingleFileProjectProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.Rust);
  }

  @Override
  public String name() {
    return "single-file-project";
  }
}
