package org.lflang.target.property;

import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

public abstract class AbstractStringConfig extends AbstractTargetProperty<String, PrimitiveType> {

  public AbstractStringConfig() {
    super(PrimitiveType.STRING);
  }

  @Override
  public String initialValue() {
    return "";
  }

  @Override
  public String fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.elementToSingleString(node);
  }

  @Override
  protected String fromString(String string, MessageReporter reporter) {
    return string;
  }

  @Override
  public Element toAstElement(String value) {
    return ASTUtils.toElement(value);
  }
}
