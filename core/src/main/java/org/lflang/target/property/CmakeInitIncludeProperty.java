package org.lflang.target.property;

import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

/**
 * Directive to specify cmake initialize files to be included at the very beginning of the
 * generated CMakeLists.txt. Here the user can override things like the toolchain file
 */

public final class CmakeInitIncludeProperty extends FileListProperty {

  /** Singleton target property instance. */
  public static final CmakeInitIncludeProperty INSTANCE = new CmakeInitIncludeProperty();

  private CmakeInitIncludeProperty() {
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
    return "cmake-init-include";
  }

  @Override
  public boolean loadFromFederate() {
    return true;
  }
}
