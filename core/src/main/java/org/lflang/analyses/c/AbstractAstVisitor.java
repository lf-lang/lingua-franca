package org.lflang.analyses.c;

import java.util.List;

/** Modeled after {@link AbstractParseTreeVisitor}. */
public abstract class AbstractAstVisitor<T> implements AstVisitor<T> {

  @Override
  public T visit(CAst.AstNode tree) {
    return tree.accept(this);
  }

  @Override
  public T visit(CAst.AstNode tree, List<CAst.AstNode> nodeList) {
    return tree.accept(this, nodeList);
  }
}
