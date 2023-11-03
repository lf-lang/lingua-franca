package org.lflang.target.property;

/**
 * If true, configure the execution environment to keep executing if there are no more events on the
 * event queue. The default is false.
 */
public final class KeepaliveProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final KeepaliveProperty INSTANCE = new KeepaliveProperty();

  private KeepaliveProperty() {
    super();
  }

  @Override
  public String name() {
    return "keepalive";
  }
}
