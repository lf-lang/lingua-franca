package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
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
    return new PlatformOptions();
  }

  @Override
  public PlatformOptions fromAst(Element node, MessageReporter reporter) {
    var config = new PlatformOptions();
    if (node.getLiteral() != null || node.getId() != null) {
      config.platform = new PlatformType().forName(ASTUtils.elementToSingleString(node));
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        PlatformOption option =
            (PlatformOption) DictionaryType.PLATFORM_DICT.forName(entry.getName());
        if (option == null) {
          continue; // FIXME: should not be necessary
        }
        switch (option) {
          case NAME -> {
            config.platform =
                new PlatformType().forName(ASTUtils.elementToSingleString(entry.getValue()));
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
    var config = fromAst(pair.getValue(), reporter);
    var threading = TargetProperty.getKeyValuePair(ast, new ThreadingProperty());
    if (threading != null && config.platform == Platform.RP2040) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("Platform " + Platform.RP2040 + " does not support threading");
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
