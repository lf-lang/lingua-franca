package org.lflang.target.property;

/**
 * If true, the RTI sends DNET signals during the execution of federation. 
 * The default is ture (for now, for testing).
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
