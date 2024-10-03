package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.PlatformProperty.PlatformOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PlatformType;
import org.lflang.target.property.type.PlatformType.Platform;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/**
 * Directive to specify the platform for cross code generation. This is either a string of the
 * platform or a dictionary of options that includes the string name.
 */
public final class PlatformProperty extends TargetProperty<PlatformOptions, UnionType> {

  /** Singleton target property instance. */
  public static final PlatformProperty INSTANCE = new PlatformProperty();

  private PlatformProperty() {
    super(UnionType.PLATFORM_STRING_OR_DICTIONARY);
  }

  @Override
  public PlatformOptions initialValue() {
    Option<String> board = new Option<String>(false, null);
    Option<String> port = new Option<String>(false, null);
    Option<Integer> baudRate = new Option<Integer>(false, 0);
    Option<Boolean> flash = new Option<Boolean>(false, false);
    Option<Integer> userThreads = new Option<Integer>(false, 0);

    return new PlatformOptions(Platform.AUTO, board, port, baudRate, flash, userThreads);
  }

  @Override
  public PlatformOptions fromAst(Element node, MessageReporter reporter) {
    var platform = Platform.AUTO;

    String boardValue = null;
    boolean boardSet = false;

    String portValue = null;
    boolean portSet = false;

    int baudRateValue = 0;
    boolean baudRateSet = false;

    boolean flashValue = false;
    boolean flashSet = false;

    int userThreadsValue = 0;
    boolean userThreadsSet = false;

    if (node.getLiteral() != null || node.getId() != null) {
      platform = new PlatformType().forName(ASTUtils.elementToSingleString(node));
    } else if (node.getKeyvalue() != null) {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        PlatformOption option =
            (PlatformOption) DictionaryType.PLATFORM_DICT.forName(entry.getName());
        if (option != null) {
          switch (option) {
            case NAME -> {
              platform =
                  new PlatformType().forName(ASTUtils.elementToSingleString(entry.getValue()));
            }
            case BAUDRATE -> {
              baudRateSet = true;
              baudRateValue = ASTUtils.toInteger(entry.getValue());
            }
            case BOARD -> {
              boardSet = true;
              boardValue = ASTUtils.elementToSingleString(entry.getValue());
            }
            case FLASH -> {
              flashSet = true;
              flashValue = ASTUtils.toBoolean(entry.getValue());
            }
            case PORT -> {
              portSet = true;
              portValue = ASTUtils.elementToSingleString(entry.getValue());
            }
            case USER_THREADS -> {
              userThreadsSet = true;
              userThreadsValue = ASTUtils.toInteger(entry.getValue());
            }
          }
        }
      }
    }

    Option<String> board = new Option<String>(boardSet, boardValue);
    Option<String> port = new Option<String>(portSet, portValue);
    Option<Integer> baudRate = new Option<Integer>(baudRateSet, baudRateValue);
    Option<Boolean> flash = new Option<Boolean>(flashSet, flashValue);
    Option<Integer> userThreads = new Option<Integer>(userThreadsSet, userThreadsValue);

    return new PlatformOptions(platform, board, port, baudRate, flash, userThreads);
  }

  @Override
  protected PlatformOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var platform = config.get(PlatformProperty.INSTANCE).platform;
    switch (platform) {
      case FLEXPRET:
        validateFlexPRET(config, reporter);
        break;
      case ZEPHYR:
        validateZephyr(config, reporter);
        break;
      default:
        break;
    }
  }

  private void validateFlexPRET(TargetConfig config, MessageReporter reporter) {
    var platform = config.get(PlatformProperty.INSTANCE);
    var board = platform.board();
    if (board.setByUser()) {
      if (!board.value().equals("emulator") && !board.value().equals("fpga")) {
        reporter
            .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
            .error(
                "Only \"emulator\" and \"fpga\" are valid options for board property. Got "
                    + board
                    + ".");
      }

      // Do validation specific to emulator
      if (board.value().equals("emulator")) {
        if (platform.port().setByUser()) {
          reporter
              .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
              .warning("Port property ignored for emulator");
        }
        if (platform.baudRate().setByUser()) {
          reporter
              .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
              .warning("Baudrate property ignored for emulator");
        }
      } else {
        if (platform.baudRate().setByUser()) {
          reporter
              .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
              .error(
                  "Baudrate property is entirely controlled by FlexPRET's SDK and cannot be set by"
                      + " the user");
        }
      }
    }
  }

  private void validateZephyr(TargetConfig config, MessageReporter reporter) {
  }

  @Override
  public Element toAstElement(PlatformOptions value) {
    Element e = LfFactory.eINSTANCE.createElement();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (PlatformOption opt : PlatformOption.values()) {
      KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
      pair.setName(opt.toString());
      switch (opt) {
        case NAME -> pair.setValue(ASTUtils.toElement(value.platform.toString()));
        case BAUDRATE -> pair.setValue(ASTUtils.toElement(value.baudRate.value));
        case BOARD -> pair.setValue(ASTUtils.toElement(value.board.value));
        case FLASH -> pair.setValue(ASTUtils.toElement(value.flash.value));
        case PORT -> pair.setValue(ASTUtils.toElement(value.port.value));
        case USER_THREADS -> pair.setValue(ASTUtils.toElement(value.userThreads.value));
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

  /** Keep track of whether a value was set by user or not. */
  public record Option<T>(boolean setByUser, T value) {}

  /** Settings related to Platform Options. */
  public record PlatformOptions(
      /**
       * The base platform we build our LF Files on. Should be set to AUTO by default unless
       * developing for specific OS/Embedded Platform
       */
      Platform platform,

      /**
       * The string value used to determine what type of embedded board we work with and can be used
       * to simplify the build process. This string has the form "board_name[:option]*" (zero or
       * more options separated by colons). For example, "pico:usb" specifies a Raspberry Pi Pico
       * where stdin and stdout go through a USB serial port.
       */
      Option<String> board,

      /**
       * The string value used to determine the port on which to flash the compiled program (i.e.
       * /dev/cu.usbmodem21301)
       */
      Option<String> port,

      /**
       * The baud rate used as a parameter to certain embedded platforms. 9600 is a standard rate
       * amongst systems like Arduino, so it's the default value.
       */
      Option<Integer> baudRate,

      /**
       * Whether we should automatically attempt to flash once we compile. This may require the use
       * of board and port values depending on the infrastructure you use to flash the boards.
       */
      Option<Boolean> flash,

      /** The number of threads requested by the user. */
      Option<Integer> userThreads) {

    public String[] boardArray() {
      // Parse board option of the platform target property
      // Specified as a series of colon spaced options
      // Board syntax
      //  rp2040 <board_name> : <stdio_opt>
      //  arduino
      String[] boardProperties = {};
      if (this.board != null) {
        boardProperties = this.board.value.trim().split(":");
        // Ignore whitespace
        for (int i = 0; i < boardProperties.length; i++) {
          boardProperties[i] = boardProperties[i].trim();
        }
      }
      return boardProperties;
    }
  }

  /**
   * Platform options.
   *
   * @author Anirudh Rengarajan
   */
  public enum PlatformOption implements DictionaryElement {
    NAME("name", new PlatformType()),
    BAUDRATE("baud-rate", PrimitiveType.NON_NEGATIVE_INTEGER),
    BOARD("board", PrimitiveType.STRING),
    FLASH("flash", PrimitiveType.BOOLEAN),
    PORT("port", PrimitiveType.STRING),
    USER_THREADS("user-threads", PrimitiveType.NON_NEGATIVE_INTEGER);

    public final TargetPropertyType type;

    private final String description;

    PlatformOption(String alias, TargetPropertyType type) {
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
