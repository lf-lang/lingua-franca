package org.lflang.target.property.type;

import org.lflang.target.property.type.CoordinationModeType.CoordinationMode;

/** Enumeration of supported platforms */
public class CoordinationModeType extends OptionsType<CoordinationMode> {

  @Override
  protected Class<CoordinationMode> enumClass() {
    return CoordinationMode.class;
  }

  /**
   * Enumeration of coordination types.
   *
   * @author Marten Lohstroh
   */
  public enum CoordinationMode {
    CENTRALIZED,
    DECENTRALIZED;

    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }

    public static CoordinationMode getDefault() {
      return CoordinationMode.CENTRALIZED;
    }
  }
}
