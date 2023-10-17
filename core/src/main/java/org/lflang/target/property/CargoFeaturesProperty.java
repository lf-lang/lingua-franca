package org.lflang.target.property;

/** Directive for specifying Cargo features of the generated program to enable. */
public final class CargoFeaturesProperty extends StringListProperty {

  /** Singleton target property instance. */
  public static final CargoFeaturesProperty INSTANCE = new CargoFeaturesProperty();

  private CargoFeaturesProperty() {
    super();
  }

  @Override
  public String name() {
    return "cargo-features";
  }
}
