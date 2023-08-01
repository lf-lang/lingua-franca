package org.lflang.analyses.c;

import java.util.List;

/**
 * An interface for Visitable classes, used for AST nodes.
 *
 * @author Shaokai Lin
 */
public interface Visitable {

  /** The {@link AstVisitor} needs a double dispatch method. */
  <T> T accept(AstVisitor<? extends T> visitor);

  /** The {@link AstVisitor} needs a double dispatch method. */
  <T> T accept(AstVisitor<? extends T> visitor, List<CAst.AstNode> nodeList);
}
