package org.lflang.target.property;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

/**
 * Directive to specify a cmake to be included by the generated build systems.
 *
 * <p>This gives full control over the C/C++ build as any cmake parameters can be adjusted in the
 * included file.
 */
public final class CmakeIncludeProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final CmakeIncludeProperty INSTANCE = new CmakeIncludeProperty();

  private CmakeIncludeProperty() {
    super();
  }

  @Override
  protected List<String> fromAst(Element node, MessageReporter reporter) {
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
    return "cmake-include";
  }

  @Override
  public boolean loadFromFederate() {
    return true;
  }
}
