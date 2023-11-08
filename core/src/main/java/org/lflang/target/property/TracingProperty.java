package org.lflang.target.property;

import java.util.Objects;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TracingProperty.TracingOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

/** Directive to configure the runtime environment to perform tracing. */
public class TracingProperty extends TargetProperty<TracingOptions, UnionType> {

  /** Singleton target property instance. */
  public static final TracingProperty INSTANCE = new TracingProperty();

  private TracingProperty() {
    super(UnionType.TRACING_UNION);
  }

  @Override
  public TracingOptions initialValue() {
    return new TracingOptions(false);
  }

  @Override
  public TracingOptions fromAst(Element node, MessageReporter reporter) {
    var options = new TracingOptions(false);
    if (node.getLiteral() != null) {
      if (ASTUtils.toBoolean(node)) {
        options.enabled = true;
      }
    } else {
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        TracingOption option = (TracingOption) DictionaryType.TRACING_DICT.forName(entry.getName());
        if (Objects.requireNonNull(option) == TracingOption.TRACE_FILE_NAME) {
          options.traceFileName = ASTUtils.elementToSingleString(entry.getValue());
        }
      }
    }
    return options;
  }

  @Override
  protected TracingOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    var noThreads =
        config.isSet(SingleThreadedProperty.INSTANCE)
            && config.get(SingleThreadedProperty.INSTANCE).equals(true);
    var pair = config.lookup(this);
    if (config.get(this).isEnabled() && noThreads) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__NAME)
          .error("Cannot enable tracing because threading support is disabled");
      reporter
          .at(config.lookup(SingleThreadedProperty.INSTANCE), Literals.KEY_VALUE_PAIR__NAME)
          .error("Cannot disable treading support because tracing is enabled");
    }
    if (pair != null) {
      if (config.target.equals(Target.CPP) && pair.getValue().getKeyvalue() != null) {
        reporter
            .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
            .warning(
                "The C++ target only supports 'true' or 'false' and ignores additional"
                    + " configuration");
      }
    }
  }

  @Override
  public Element toAstElement(TracingOptions value) {
    if (!value.isEnabled()) {
      return null;
    } else if (value.equals(new TracingOptions(true))) {
      // default values
      return ASTUtils.toElement(true);
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (TracingOption opt : TracingOption.values()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(opt.toString());
        if (opt == TracingOption.TRACE_FILE_NAME) {
          if (value.traceFileName == null) {
            continue;
          }
          pair.setValue(ASTUtils.toElement(value.traceFileName));
        }
        kvp.getPairs().add(pair);
      }
      e.setKeyvalue(kvp);
      if (kvp.getPairs().isEmpty()) {
        return null;
      }
      return e;
    }
  }

  @Override
  public String name() {
    return "tracing";
  }

  @Override
  public void update(TargetConfig config, TracingOptions value) {
    if (value.traceFileName == null) {
      value.traceFileName = config.get(this).traceFileName;
    }
    config.set(this, value);
  }

  /** Settings related to tracing options. */
  public static class TracingOptions {

    protected boolean enabled;

    public TracingOptions(boolean enabled) {
      this.enabled = enabled;
    }

    /**
     * The name to use as the root of the trace file produced. This defaults to the name of the .lf
     * file.
     */
    public String traceFileName = null;

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if (o == null || getClass() != o.getClass()) {
        return false;
      }
      TracingOptions that = (TracingOptions) o;
      return Objects.equals(traceFileName, that.traceFileName); // traceFileName may be null
    }

    public boolean isEnabled() {
      return enabled;
    }
  }

  /**
   * Tracing options.
   *
   * @author Edward A. Lee
   */
  public enum TracingOption implements DictionaryElement {
    TRACE_FILE_NAME("trace-file-name", PrimitiveType.STRING);

    public final PrimitiveType type;

    private final String description;

    TracingOption(String alias, PrimitiveType type) {
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
