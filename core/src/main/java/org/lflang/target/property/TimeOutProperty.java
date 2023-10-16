package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.PrimitiveType;

/** The timeout to be observed during execution of the program. */
public class TimeOutProperty extends TargetProperty<TimeValue, PrimitiveType> {

  public TimeOutProperty() {
    super(PrimitiveType.TIME_VALUE);
  }

  @Override
  public TimeValue initialValue() {
    return null;
  }

  @Override
  public TimeValue fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.toTimeValue(node);
  }

  @Override
  protected TimeValue fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Target.ALL;
  }

  @Override
  public Element toAstElement(TimeValue value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "timeout";
  }
}
