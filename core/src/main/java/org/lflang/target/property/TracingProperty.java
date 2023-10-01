package org.lflang.target.property;

import java.util.List;
import java.util.Objects;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.target.property.TracingProperty.TracingOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.UnionType;

public class TracingProperty extends AbstractTargetProperty<TracingOptions> {

  public TracingProperty() {
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
        switch (option) {
          case TRACE_FILE_NAME:
            options.traceFileName = ASTUtils.elementToSingleString(entry.getValue());
            break;
          default:
            break;
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
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.CPP, Target.Python);
  }

  @Override
  public void validate(KeyValuePair pair, Model ast, MessageReporter reporter) {
    if (pair != null && this.fromAst(pair.getValue(), reporter) != null) {
      // If tracing is anything but "false" and threading is off, error.
      var threading = TargetProperty.getKeyValuePair(ast, TargetProperty.THREADING);
      if (threading != null) {
        if (!ASTUtils.toBoolean(threading.getValue())) {
          reporter
              .at(pair, Literals.KEY_VALUE_PAIR__NAME)
              .error("Cannot enable tracing because threading support is disabled");
          reporter
              .at(threading, Literals.KEY_VALUE_PAIR__NAME)
              .error("Cannot disable treading support because tracing is enabled");
        }
      }
    }
  }

  @Override
  public Element toAstElement() {
    if (!this.get().isEnabled()) {
      return null;
    } else if (this.get().equals(new TracingOptions(true))) {
      // default values
      return ASTUtils.toElement(true);
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (TracingOption opt : TracingOption.values()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(opt.toString());
        switch (opt) {
          case TRACE_FILE_NAME:
            if (this.get().traceFileName == null) {
              continue;
            }
            pair.setValue(ASTUtils.toElement(this.get().traceFileName));
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

  /** Settings related to tracing options. */
  public static class TracingOptions {

    protected boolean enabled = false;

    TracingOptions(boolean enabled) {
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

    private TracingOption(String alias, PrimitiveType type) {
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
