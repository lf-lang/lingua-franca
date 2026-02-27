package org.lflang.ast;

import java.util.List;
import org.lflang.lf.Reactor;

/**
 * Interface for AST Transfomations
 *
 * @ingroup Utilities
 */
public interface AstTransformation {

  /** Apply the AST transformation to all given reactors. */
  void applyTransformation(List<Reactor> reactors);
}
