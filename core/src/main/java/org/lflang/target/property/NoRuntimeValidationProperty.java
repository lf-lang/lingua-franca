package org.lflang.target.property;

import java.util.List;
import org.lflang.Target;

/** If true, do not perform runtime validation. The default is false. */
public final class NoRuntimeValidationProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final NoRuntimeValidationProperty INSTANCE = new NoRuntimeValidationProperty();

  private NoRuntimeValidationProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.CPP);
  }

  @Override
  public String name() {
    return "no-runtime-validation";
  }
}
