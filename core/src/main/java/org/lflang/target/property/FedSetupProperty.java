package org.lflang.target.property;

import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.util.StringUtil;

public class FedSetupProperty extends AbstractTargetProperty<String> {

  public FedSetupProperty() {
    super(PrimitiveType.FILE);
  }

  @Override
  public String initialValue() {
    return null;
  }

  @Override
  protected String fromAst(Element node, MessageReporter reporter) {
    return StringUtil.removeQuotes(ASTUtils.elementToSingleString(node));
  }

  @Override
  protected String fromString(String string, MessageReporter reporter) {
    return string;
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(get());
  }

  @Override
  public String name() {
    return "_fed_setup"; // FIXME: follow kebab case convention
  }
}
