package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

public abstract class BooleanProperty extends TargetProperty<Boolean, PrimitiveType> {

  protected BooleanProperty() {
    super(PrimitiveType.BOOLEAN);
  }

  @Override
  public Boolean initialValue() {
    return false;
  }

  @Override
  public Boolean fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.toBoolean(node);
  }

  @Override
  protected Boolean fromString(String string, MessageReporter reporter) {
    return Boolean.parseBoolean(string);
  }

  @Override
  public Element toAstElement(Boolean value) {
    return ASTUtils.toElement(value);
  }
}
