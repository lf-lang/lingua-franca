package org.lflang.target.property;

import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

public class WorkersProperty extends AbstractTargetProperty<Integer> {

  public WorkersProperty() {
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
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python, Target.CPP, Target.Rust);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(get());
  }

  @Override
  public String name() {
    return "workers";
  }
}
