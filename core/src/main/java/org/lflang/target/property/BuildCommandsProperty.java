package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public class BuildCommandsProperty extends TargetPropertyConfig<List<String>> {

  public BuildCommandsProperty() {
    super(UnionType.STRING_OR_STRING_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public List<String> fromAst(Element value, MessageReporter err) {
    return ASTUtils.elementToListOfStrings(value);
  }

  @Override
  protected List<String> fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.value.toString());
  }
}
