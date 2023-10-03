package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.TargetProperty;
import org.lflang.target.property.PlatformProperty.PlatformOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/**
 * Directive to specify the platform for cross code generation. This is either a string of the
 * platform or a dictionary of options that includes the string name.
 */
public class PlatformProperty extends AbstractTargetProperty<PlatformOptions> {

  public static final String UNKNOW_PLATFORM =
      "Unidentified Platform Type, LF supports the following platform types: "
          + Arrays.asList(Platform.values());

  public PlatformProperty() {
    super(UnionType.PLATFORM_STRING_OR_DICTIONARY);
  }

  @Override
  public PlatformOptions initialValue() {
    return new PlatformOptions();
  }

  @Override
  public PlatformOptions fromAst(Element node, MessageReporter reporter) {
    var config = new PlatformOptions();
    if (node.getLiteral() != null || node.getId() != null) {
      config.platform =
          (Platform) UnionType.PLATFORM_UNION.forName(ASTUtils.elementToSingleString(node));
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        PlatformOption option =
            (PlatformOption) DictionaryType.PLATFORM_DICT.forName(entry.getName());
        switch (option) {
          case NAME -> {
            config.platform =
                (Platform)
                    UnionType.PLATFORM_UNION.forName(
                        ASTUtils.elementToSingleString(entry.getValue()));
          }
          case BAUDRATE -> config.baudRate = ASTUtils.toInteger(entry.getValue());
          case BOARD -> config.board = ASTUtils.elementToSingleString(entry.getValue());
          case FLASH -> config.flash = ASTUtils.toBoolean(entry.getValue());
          case PORT -> config.port = ASTUtils.elementToSingleString(entry.getValue());
          case USER_THREADS -> config.userThreads = ASTUtils.toInteger(entry.getValue());
        }
      }
    }

    return config;
  }

  @Override
  protected PlatformOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C);
  }

  @Override
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {
    final var node = pair.getValue();
    Platform platform = null;
    if (node.getLiteral() != null || node.getId() != null) {
      platform = (Platform) UnionType.PLATFORM_UNION.forName(ASTUtils.elementToSingleString(node));
      if (platform == null) {
        reporter.at(pair, Literals.KEY_VALUE_PAIR__VALUE).error(UNKNOW_PLATFORM);
      }
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        PlatformOption option =
            (PlatformOption) DictionaryType.PLATFORM_DICT.forName(entry.getName());
        if (Objects.requireNonNull(option) == PlatformOption.NAME) {
          platform =
              (Platform)
                  UnionType.PLATFORM_UNION.forName(
                      ASTUtils.elementToSingleString(entry.getValue()));
          if (platform == null) {
            reporter.at(entry, Literals.KEY_VALUE_PAIR__VALUE).error(UNKNOW_PLATFORM);
          }
        }
      }
    }

    var threading = TargetProperty.getKeyValuePair(ast, new ThreadingProperty());
    if (threading != null && platform == Platform.RP2040) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("Platform " + Platform.RP2040 + " does not support threading");
    }
  }

  @Override
  public Element toAstElement() {
    Element e = LfFactory.eINSTANCE.createElement();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (PlatformOption opt : PlatformOption.values()) {
      KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
      pair.setName(opt.toString());
      switch (opt) {
        case NAME -> pair.setValue(ASTUtils.toElement(get().platform.toString()));
        case BAUDRATE -> pair.setValue(ASTUtils.toElement(get().baudRate));
        case BOARD -> pair.setValue(ASTUtils.toElement(get().board));
        case FLASH -> pair.setValue(ASTUtils.toElement(get().flash));
        case PORT -> pair.setValue(ASTUtils.toElement(get().port));
        case USER_THREADS -> pair.setValue(ASTUtils.toElement(get().userThreads));
      }
      kvp.getPairs().add(pair);
    }
    e.setKeyvalue(kvp);
    if (kvp.getPairs().isEmpty()) {
      return null;
    }
    return e;
  }

  @Override
  public String name() {
    return "platform";
  }

  /** Settings related to Platform Options. */
  public static class PlatformOptions { // FIXME: use a record for this

    /**
     * The base platform we build our LF Files on. Should be set to AUTO by default unless
     * developing for specific OS/Embedded Platform
     */
    public Platform platform = Platform.AUTO;

    /**
     * The string value used to determine what type of embedded board we work with and can be used
     * to simplify the build process. This string has the form "board_name[:option]*" (zero or more
     * options separated by colons). For example, "pico:usb" specifies a Raspberry Pi Pico where
     * stdin and stdout go through a USB serial port.
     */
    public String board = null;

    /**
     * The string value used to determine the port on which to flash the compiled program (i.e.
     * /dev/cu.usbmodem21301)
     */
    public String port = null;

    /**
     * The baud rate used as a parameter to certain embedded platforms. 9600 is a standard rate
     * amongst systems like Arduino, so it's the default value.
     */
    public int baudRate = 9600;

    /**
     * The boolean statement used to determine whether we should automatically attempt to flash once
     * we compile. This may require the use of board and port values depending on the infrastructure
     * you use to flash the boards.
     */
    public boolean flash = false;

    /**
     * The int value is used to determine the number of needed threads for the user application in
     * Zephyr.
     */
    public int userThreads = 0;
  }

  /** Enumeration of supported platforms */
  public enum Platform {
    AUTO,
    ARDUINO,
    NRF52("Nrf52", true),
    RP2040("Rp2040", false),
    LINUX("Linux", true),
    MAC("Darwin", true),
    ZEPHYR("Zephyr", true),
    WINDOWS("Windows", true);

    final String cMakeName;

    private boolean multiThreaded =
        true; // FIXME: this is never read. If we set it, we can simplify the validator method in
    // the encapsulating class.

    Platform() {
      this.cMakeName = this.toString();
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
  }

  /**
   * Platform options.
   *
   * @author Anirudh Rengarajan
   */
  public enum PlatformOption implements DictionaryElement {
    NAME("name", PrimitiveType.STRING),
    BAUDRATE("baud-rate", PrimitiveType.NON_NEGATIVE_INTEGER),
    BOARD("board", PrimitiveType.STRING),
    FLASH("flash", PrimitiveType.BOOLEAN),
    PORT("port", PrimitiveType.STRING),
    USER_THREADS("user-threads", PrimitiveType.NON_NEGATIVE_INTEGER);

    public final PrimitiveType type;

    private final String description;

    PlatformOption(String alias, PrimitiveType type) {
      this.description = alias;
      this.type = type;
    }

    /** Return the description of this dictionary element. */
    @Override
    public String toString() {
      return this.description;
    }

    /** Return the type associated with this dictionary element. */
    public TargetPropertyType getType() {
      return this.type;
    }
  }
}
