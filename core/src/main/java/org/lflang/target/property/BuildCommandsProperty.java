package org.lflang.target.property;

import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

/**
 * A list of custom build commands that replace the default build process of directly invoking a
 * designated compiler. A common usage of this target property is to set the command to build on the
 * basis of a Makefile.
 */
public final class BuildCommandsProperty extends TargetProperty<List<String>, UnionType> {

  /** Singleton target property instance. */
  public static final BuildCommandsProperty INSTANCE = new BuildCommandsProperty();

  private BuildCommandsProperty() {
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
  public Element toAstElement(List<String> value) {
    return ASTUtils.toElement(value);
  }

  @Override
  public String name() {
    return "build";
  }
}
