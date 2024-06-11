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
    NRF52("nRF52", false),
    RP2040("Rp2040", true),
    LINUX("Linux", true),
    MAC("Darwin", true),
    ZEPHYR("Zephyr", true),
    FLEXPRET("FlexPRET", true),
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
