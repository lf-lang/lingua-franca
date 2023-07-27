package org.lflang.analyses.c;

import java.util.List;

/** Modeled after ParseTreeVisitor.class */
public interface AstVisitor<T> {

  /**
   * Visit an AST, and return a user-defined result of the operation.
   *
   * @param tree The {@link CAst.AstNode} to visit.
   * @return The result of visiting the parse tree.
   */
  T visit(CAst.AstNode tree);

  /**
   * Visit an AST with a list of other AST nodes holding some information, and return a user-defined
   * result of the operation.
   *
   * @param tree The {@link CAst.AstNode} to visit.
   * @param nodeList A list of {@link CAst.AstNode} passed down the recursive call.
   * @return The result of visiting the parse tree.
   */
  T visit(CAst.AstNode tree, List<CAst.AstNode> nodeList);
}
