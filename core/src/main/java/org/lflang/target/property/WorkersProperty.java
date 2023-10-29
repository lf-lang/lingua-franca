package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.TargetProperty;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.GeneratorArguments;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

/**
 * The number of worker threads to deploy. The default is zero, which indicates that the runtime is
 * allowed to freely choose the number of workers.
 */
public final class WorkersProperty extends TargetProperty<Integer, PrimitiveType> {

  /** Singleton target property instance. */
  public static final WorkersProperty INSTANCE = new WorkersProperty();

  private WorkersProperty() {
    super(PrimitiveType.NON_NEGATIVE_INTEGER);
  }

  @Override
  public Integer initialValue() {
    return 0;
  }

  @Override
  protected Integer fromString(String string, MessageReporter reporter) {
    return Integer.parseInt(string); // FIXME: check for exception
  }

  @Override
  protected Integer fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.toInteger(node);
  }

  @Override
  public Element toAstElement(Integer value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "workers";
  }

  @Override
  public Integer value(GeneratorArguments args) {
    return args.workers;
  }
}
