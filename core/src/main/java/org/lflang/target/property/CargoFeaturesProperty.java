package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import org.lflang.Target;

/** Directive for specifying Cargo features of the generated program to enable. */
public class CargoFeaturesProperty extends StringListProperty {

  /** Singleton target property instance. */
  public static final CargoFeaturesProperty INSTANCE = new CargoFeaturesProperty();

  private CargoFeaturesProperty() {
    super();
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.Rust);
  }

  @Override
  public String name() {
    return "cargo-features";
  }
}
