package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TracePluginProperty.TracePluginOptions;
import org.lflang.target.property.type.PrimitiveType;

/** Property that provides an alternative tracing implementation. */
public class TracePluginProperty extends TargetProperty<TracePluginOptions, PrimitiveType> {

  /** Singleton target property instance. */
  public static final TracePluginProperty INSTANCE = new TracePluginProperty();

  private TracePluginProperty() {
    super(PrimitiveType.STRING);
  }

  @Override
  public TracePluginOptions initialValue() {
    return null;
  }

  @Override
  protected TracePluginOptions fromAst(Element node, MessageReporter reporter) {
    var s = ASTUtils.elementToSingleString(node);
    if (s != null) {
      return new TracePluginOptions(s);
    } else {
      reporter.at(node).error("Expected a string literal");
      return null;
    }
  }

  @Override
  protected TracePluginOptions fromString(String s, MessageReporter reporter) {
    return new TracePluginOptions(s);
  }

  @Override
  public String name() {
    return "trace-plugin";
  }

  @Override
  public Element toAstElement(TracePluginOptions value) {
    throw new UnsupportedOperationException(TargetConfig.NOT_IN_LF_SYNTAX_MESSAGE);
  }

  public static class TracePluginOptions {
    private final String implementationArchiveFile;

    public TracePluginOptions(String implementationArchiveFile) {
      this.implementationArchiveFile = implementationArchiveFile;
    }

    public String getImplementationArchiveFile() {
      return implementationArchiveFile;
    }
  }
}
