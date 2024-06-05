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
   * The values associated with the enum must match what is defined in clock_sync.h in reactor-c
   *
   * @author Edward A. Lee
   */
  public enum ClockSyncMode {
    OFF(1),
    INIT(2),
    ON(3);
    private final int value;

    private ClockSyncMode(int value) {
      this.value = value;
    }

    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }

    public int toInt() {
      return this.value;
    }
  }
}
