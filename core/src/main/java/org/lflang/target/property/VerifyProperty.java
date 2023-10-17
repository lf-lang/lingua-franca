package org.lflang.target.property.type;

import org.lflang.target.property.BooleanProperty;

/** If true, check the generated verification model. The default is false. */
public final class VerifyProperty extends BooleanProperty {

  /** Singleton target property instance. */
  public static final VerifyProperty INSTANCE = new VerifyProperty();

  private VerifyProperty() {
    super();
  }

  @Override
  public String name() {
    return "verify";
  }
}
