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
    return new PlatformOptions(Platform.AUTO, null, null, 9600, false, 0);
  }

  @Override
  public PlatformOptions fromAst(Element node, MessageReporter reporter) {
    var platform = Platform.AUTO;
    String board = null;
    String port = null;
    var baudRate = 9600;
    var flash = false;
    var userThreads = 0;
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
            case BAUDRATE -> baudRate = ASTUtils.toInteger(entry.getValue());
            case BOARD -> board = ASTUtils.elementToSingleString(entry.getValue());
            case FLASH -> flash = ASTUtils.toBoolean(entry.getValue());
            case PORT -> port = ASTUtils.elementToSingleString(entry.getValue());
            case USER_THREADS -> userThreads = ASTUtils.toInteger(entry.getValue());
          }
        }
      }
    }
    return new PlatformOptions(platform, board, port, baudRate, flash, userThreads);
  }

  @Override
  protected PlatformOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var singleThreaded = config.get(SingleThreadedProperty.INSTANCE);
    if (!singleThreaded && !config.get(PlatformProperty.INSTANCE).platform.isMultiThreaded()) {
      reporter
          .at(config.lookup(this), Literals.KEY_VALUE_PAIR__VALUE)
          .error("Platform " + config.get(PlatformProperty.INSTANCE).platform + " does not support threading.");
    }
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
        case BAUDRATE -> pair.setValue(ASTUtils.toElement(value.baudRate));
        case BOARD -> pair.setValue(ASTUtils.toElement(value.board));
        case FLASH -> pair.setValue(ASTUtils.toElement(value.flash));
        case PORT -> pair.setValue(ASTUtils.toElement(value.port));
        case USER_THREADS -> pair.setValue(ASTUtils.toElement(value.userThreads));
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
      String board,

      /**
       * The string value used to determine the port on which to flash the compiled program (i.e.
       * /dev/cu.usbmodem21301)
       */
      String port,

      /**
       * The baud rate used as a parameter to certain embedded platforms. 9600 is a standard rate
       * amongst systems like Arduino, so it's the default value.
       */
      int baudRate,

      /**
       * Whether we should automatically attempt to flash once we compile. This may require the use
       * of board and port values depending on the infrastructure you use to flash the boards.
       */
      boolean flash,

      /**
       * The baud rate used as a parameter to certain embedded platforms. 9600 is a standard rate
       * amongst systems like Arduino, so it's the default value.
       */
      int userThreads) {

    public String[] boardArray() {
      // Parse board option of the platform target property
      // Specified as a series of colon spaced options
      // Board syntax
      //  rp2040 <board_name> : <stdio_opt>
      //  arduino
      String[] boardProperties = {};
      if (this.board != null) {
        boardProperties = this.board.trim().split(":");
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
