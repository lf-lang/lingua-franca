package org.lflang.target.property.type;

import org.lflang.target.property.type.ClockSyncModeType.ClockSyncMode;

public class ClockSyncModeType extends OptionsType<ClockSyncMode> {

  @Override
  protected Class<ClockSyncMode> enumClass() {
    return ClockSyncMode.class;
  }

  /**
   * Enumeration of clock synchronization modes.
   *
   * <ul>
   *   <li>OFF: The clock synchronization is universally off.
   *   <li>STARTUP: Clock synchronization occurs at startup only.
   *   <li>ON: Clock synchronization occurs at startup and at runtime.
   * </ul>
   *
   * @author Edward A. Lee
   */
  public enum ClockSyncMode {
    OFF,
    INIT,
    ON;
    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }
  }
}
