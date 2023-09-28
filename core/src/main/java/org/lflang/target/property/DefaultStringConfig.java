package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

public abstract class DefaultStringConfig extends TargetPropertyConfig<String> {

  public DefaultStringConfig() {
    super(PrimitiveType.STRING);
  }

  @Override
  public String initialValue() {
    return "";
  }

  @Override
  public String fromAst(Element value, MessageReporter err) {
    return ASTUtils.elementToSingleString(value);
  }

  @Override
  protected String fromString(String value, MessageReporter err) {
    return value;
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(value);
  }
}
