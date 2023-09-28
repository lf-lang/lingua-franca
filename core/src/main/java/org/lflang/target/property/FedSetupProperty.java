package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.util.StringUtil;

public class FedSetupProperty extends TargetPropertyConfig<String> {

  public FedSetupProperty() {
    super(PrimitiveType.FILE);
  }

  @Override
  public String initialValue() {
    return null;
  }

  @Override
  protected String fromAst(Element value, MessageReporter err) {
    return StringUtil.removeQuotes(ASTUtils.elementToSingleString(value));
  }

  @Override
  protected String fromString(String value, MessageReporter err) {
    return value;
  }

  @Override
  public List<Target> supportedTargets() {
    return List.of(Target.C, Target.CCPP, Target.Python);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(value);
  }
}
