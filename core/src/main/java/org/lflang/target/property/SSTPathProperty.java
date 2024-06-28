package org.lflang.target.property;

/** The compiler to invoke, unless a build command has been specified. */
public final class SSTPathProperty extends StringProperty {

  /** Singleton target property instance. */
  public static final SSTPathProperty INSTANCE = new SSTPathProperty();

  private SSTPathProperty() {
    super();
  }

  @Override
  public String name() {
    return "sst-root-path";
  }
}
