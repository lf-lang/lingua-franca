package org.lflang.target.property;

/**
 * @brief Target property turning on or off the DNET signal optimization.
 * 
 * If this target property is true, the RTI sends DNET (downstream next event tag) signals to an upstream federate
 * to tell the federate that sending LTC and NET signals with tags less than the specified value is unnecessary.
 * The default is true.
 */
public final class DNETProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final DNETProperty INSTANCE = new DNETProperty();

  private DNETProperty() {
    super();
  }

  @Override
  public Boolean initialValue() {
    return true;
  }

  @Override
  public String name() {
    return "DNET";
  }
}
