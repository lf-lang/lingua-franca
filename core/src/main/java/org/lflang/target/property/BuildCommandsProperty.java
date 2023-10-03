package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

/** Directive to let the generator use the custom build command. */
public class BuildCommandsProperty extends AbstractTargetProperty<List<String>> {

  public BuildCommandsProperty() {
    super(UnionType.STRING_OR_STRING_ARRAY);
  }

  @Override
  public List<String> initialValue() {
    return new ArrayList<>();
  }

  @Override
  public List<String> fromAst(Element node, MessageReporter reporter) {
    return ASTUtils.elementToListOfStrings(node);
  }

  @Override
  protected List<String> fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.C, Target.CCPP);
  }

  @Override
  public Element toAstElement() {
    return ASTUtils.toElement(this.get().toString());
  }

  @Override
  public String name() {
    return "build";
  }
}
