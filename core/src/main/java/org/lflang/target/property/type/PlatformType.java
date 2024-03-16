package org.lflang.target.property.type;

import org.lflang.target.property.type.PlatformType.Platform;

/** Enumeration of supported platforms */
public class PlatformType extends OptionsType<Platform> {

  @Override
  protected Class<Platform> enumClass() {
    return Platform.class;
  }

  public enum Platform {
    AUTO,
    ARDUINO, // FIXME: not multithreaded
    NRF52("Nrf52", true),
    RP2040("Rp2040", false),
    LINUX("Linux", true),
    MAC("Darwin", true),
    ZEPHYR("Zephyr", true),
    STM32("STM32", false),
    WINDOWS("Windows", true);

    final String cMakeName;

    private final boolean multiThreaded;

    Platform() {
      this.cMakeName = this.toString();
      this.multiThreaded = true;
    }

    Platform(String cMakeName, boolean isMultiThreaded) {
      this.cMakeName = cMakeName;
      this.multiThreaded = isMultiThreaded;
    }

    /** Return the name in lower case. */
    @Override
    public String toString() {
      return this.name().toLowerCase();
    }

    /** Get the CMake name for the platform. */
    public String getcMakeName() {
      return this.cMakeName;
    }

    public boolean isMultiThreaded() {
      return this.multiThreaded;
    }

    public Platform getDefault() {
      return Platform.AUTO;
    }
  }
}
