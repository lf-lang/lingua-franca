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
<<<<<<< HEAD
    ARDUINO, // FIXME: not multithreaded
    NRF52("nRF52", false),
    RP2040("Rp2040", true),
    LINUX("Linux", true),
    MAC("Darwin", true),
    ZEPHYR("Zephyr", true),
    STM32("Stm32", false),
    FLEXPRET("FlexPRET", true),
    WINDOWS("Windows", true);
=======
    ARDUINO,
    NRF52("nRF52"),
    RP2040("Rp2040"),
    LINUX("Linux"),
    MAC("Darwin"),
    ZEPHYR("Zephyr"),
    FLEXPRET("FlexPRET"),
    WINDOWS("Windows");
>>>>>>> master

    final String cMakeName;

    Platform() {
      this.cMakeName = this.toString();
    }

    Platform(String cMakeName) {
      this.cMakeName = cMakeName;
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

    public Platform getDefault() {
      return Platform.AUTO;
    }

    /** Return {@code true} if the given platform supports federated. */
    public static boolean supportsFederated(Platform platform) {
      return switch (platform) {
        case AUTO, LINUX, MAC -> true;
        default -> false;
      };
    }
  }
}
