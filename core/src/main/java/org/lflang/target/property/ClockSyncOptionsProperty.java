package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.ClockSyncOptionsProperty.ClockSyncOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;

/** Key-value pairs giving options for clock synchronization. */
public final class ClockSyncOptionsProperty
    extends TargetProperty<ClockSyncOptions, DictionaryType> {

  /** Singleton target property instance. */
  public static final ClockSyncOptionsProperty INSTANCE = new ClockSyncOptionsProperty();

  private ClockSyncOptionsProperty() {
    super(DictionaryType.CLOCK_SYNC_OPTION_DICT);
  }

  @Override
  public ClockSyncOptions initialValue() {
    return new ClockSyncOptions();
  }

  @Override
  public ClockSyncOptions fromAst(Element node, MessageReporter reporter) {
    var options = new ClockSyncOptions();
    for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
      ClockSyncOption option =
          (ClockSyncOption) DictionaryType.CLOCK_SYNC_OPTION_DICT.forName(entry.getName());
      if (option != null) {
        switch (option) {
          case ATTENUATION -> options.attenuation = ASTUtils.toInteger(entry.getValue());
          case COLLECT_STATS -> options.collectStats = ASTUtils.toBoolean(entry.getValue());
          case LOCAL_FEDERATES_ON -> options.localFederatesOn =
              ASTUtils.toBoolean(entry.getValue());
          case PERIOD -> options.period = ASTUtils.toTimeValue(entry.getValue());
          case TEST_OFFSET -> options.testOffset = ASTUtils.toTimeValue(entry.getValue());
          case TRIALS -> options.trials = ASTUtils.toInteger(entry.getValue());
          default -> {}
        }
      }
    }
    return options;
  }

  @Override
  protected ClockSyncOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement(ClockSyncOptions value) {
    Element e = LfFactory.eINSTANCE.createElement();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (ClockSyncOption opt : ClockSyncOption.values()) {
      KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
      pair.setName(opt.toString());
      switch (opt) {
        case ATTENUATION -> pair.setValue(ASTUtils.toElement(value.attenuation));
        case COLLECT_STATS -> pair.setValue(ASTUtils.toElement(value.collectStats));
        case LOCAL_FEDERATES_ON -> pair.setValue(ASTUtils.toElement(value.localFederatesOn));
        case PERIOD -> {
          if (value.period == null) {
            continue; // don't set if null
          }
          pair.setValue(ASTUtils.toElement(value.period));
        }
        case TEST_OFFSET -> {
          if (value.testOffset == null) {
            continue; // don't set if null
          }
          pair.setValue(ASTUtils.toElement(value.testOffset));
        }
        case TRIALS -> pair.setValue(ASTUtils.toElement(value.trials));
      }
      kvp.getPairs().add(pair);
    }
    e.setKeyvalue(kvp);
    // kvp will never be empty
    return e;
  }

  @Override
  public String name() {
    return "clock-sync-options";
  }

  /** Settings related to clock synchronization. */
  public static class ClockSyncOptions {

    /**
     * Dampen the adjustments to the clock synchronization offset by this rate. The default is 10.
     */
    public int attenuation = 10;

    /**
     * Whether to collect statistics while performing clock synchronization. This setting is only
     * considered when clock synchronization has been activated. The default is true.
     */
    public boolean collectStats = true;

    /** Enable clock synchronization for federates on the same machine. Default is false. */
    public boolean localFederatesOn = false;

    /**
     * Interval at which clock synchronization is initiated by the RTI (will be passed to it as an
     * argument on the command-line). The default is 5 milliseconds.
     */
    public TimeValue period = new TimeValue(5, TimeUnit.MILLI);

    /**
     * Indicate the number of exchanges to be had per each clock synchronization round. See
     * /lib/core/federated/clock-sync.h for more details. The default is 10.
     */
    public int trials = 10;

    /**
     * Used to create an artificial clock synchronization error for the purpose of testing. The
     * default is null.
     */
    public TimeValue testOffset;
  }

  /**
   * Clock synchronization options.
   *
   * @author Marten Lohstroh
   */
  public enum ClockSyncOption implements DictionaryElement {
    ATTENUATION("attenuation", PrimitiveType.NON_NEGATIVE_INTEGER),
    LOCAL_FEDERATES_ON("local-federates-on", PrimitiveType.BOOLEAN),
    PERIOD("period", PrimitiveType.TIME_VALUE),
    TEST_OFFSET("test-offset", PrimitiveType.TIME_VALUE),
    TRIALS("trials", PrimitiveType.NON_NEGATIVE_INTEGER),
    COLLECT_STATS("collect-stats", PrimitiveType.BOOLEAN);

    public final PrimitiveType type;

    private final String description;

    ClockSyncOption(String alias, PrimitiveType type) {
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
