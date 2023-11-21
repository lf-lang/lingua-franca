package org.lflang.target.property;

/**
 * If true, the RTI will send NDT messages to federates to reduce the number of messages.
 * The default is false.
 */
public final class NDTProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final NDTProperty INSTANCE = new NDTProperty();

  private NDTProperty() {
    super();
  }

  @Repeatatable
  public Boolean initialValue() {
    return false;
  }

  @Override
  public String name() {
    return "ndt";
  }
}
