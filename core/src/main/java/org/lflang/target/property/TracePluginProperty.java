package org.lflang.target.property;

import java.util.Objects;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;

/** Property that provides an alternative tracing implementation via a CMake package + target. */
public final class TracePluginProperty
    extends TargetProperty<TracePluginProperty.TracePluginSpec, DictionaryType> {

  /** Singleton target property instance. */
  public static final TracePluginProperty INSTANCE = new TracePluginProperty();

  private TracePluginProperty() {
    super(DictionaryType.TRACE_PLUGIN_DICT);
  }

  /** Parsed value of the trace-plugin target property. */
  public static final class TracePluginSpec {
    /** CMake package name used for find_package(); becomes LF_TRACE_PLUGIN. */
    public String pkg;

    /** CMake target to link; becomes LF_TRACE_PLUGIN_LIBRARY. */
    public String library;

    /**
     * Optional CMake search paths passed as LF_TRACE_PLUGIN_PATHS.
     *
     * <p>This should be a semicolon-separated list of paths (CMake list syntax), e.g.
     * {@code /opt/myplugin;/home/me/myplugin/install}.
     */
    public String paths;
  }

  @Override
  public TracePluginSpec initialValue() {
    return null; // property is unset by default
  }

  @Override
  protected TracePluginSpec fromAst(Element node, MessageReporter reporter) {
    var spec = new TracePluginSpec();
    for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
      TracePluginOption opt =
          (TracePluginOption) DictionaryType.TRACE_PLUGIN_DICT.forName(entry.getName());
      switch (Objects.requireNonNull(opt)) {
        case PACKAGE -> spec.pkg = ASTUtils.elementToSingleString(entry.getValue());
        case LIBRARY -> spec.library = ASTUtils.elementToSingleString(entry.getValue());
        case PATH -> spec.paths = ASTUtils.elementToSingleString(entry.getValue());
      }
    }
    return spec;
  }

  @Override
  protected TracePluginSpec fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException(
        "trace-plugin no longer accepts a string; use a dictionary with keys 'package' and"
            + " 'library'.");
  }

  @Override
  public void validate(TargetConfig config, MessageReporter reporter) {
    if (!config.isSet(this)) return;
    var pair = config.lookup(this);
    var spec = config.get(this);
    if (spec == null
        || spec.pkg == null
        || spec.pkg.isBlank()
        || spec.library == null
        || spec.library.isBlank()) {
      reporter
          .at(pair, Literals.KEY_VALUE_PAIR__VALUE)
          .error("trace-plugin must be a dictionary with keys 'package' and 'library'.");
    }
  }

  @Override
  public Element toAstElement(TracePluginSpec value) {
    if (value == null) return null;
    if (value.pkg == null && value.library == null) return null;

    Element e = LfFactory.eINSTANCE.createElement();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();

    if (value.pkg != null) {
      KeyValuePair p = LfFactory.eINSTANCE.createKeyValuePair();
      p.setName(TracePluginOption.PACKAGE.toString());
      p.setValue(ASTUtils.toElement(value.pkg));
      kvp.getPairs().add(p);
    }

    if (value.library != null) {
      KeyValuePair p = LfFactory.eINSTANCE.createKeyValuePair();
      p.setName(TracePluginOption.LIBRARY.toString());
      p.setValue(ASTUtils.toElement(value.library));
      kvp.getPairs().add(p);
    }

    if (value.paths != null) {
      KeyValuePair p = LfFactory.eINSTANCE.createKeyValuePair();
      p.setName(TracePluginOption.PATH.toString());
      p.setValue(ASTUtils.toElement(value.paths));
      kvp.getPairs().add(p);
    }

    e.setKeyvalue(kvp);
    return kvp.getPairs().isEmpty() ? null : e;
  }

  @Override
  public String name() {
    return "trace-plugin";
  }

  public enum TracePluginOption implements DictionaryElement {
    PACKAGE("package", PrimitiveType.STRING),
    LIBRARY("library", PrimitiveType.STRING),
    PATH("path", PrimitiveType.STRING);

    private final String description;
    private final PrimitiveType type;

    TracePluginOption(String description, PrimitiveType type) {
      this.description = description;
      this.type = type;
    }

    @Override
    public String toString() {
      return description;
    }

    @Override
    public TargetPropertyType getType() {
      return type;
    }
  }
}
