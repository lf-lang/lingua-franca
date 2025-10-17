package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;

/**
 * An exception that indicates invalid source, which should be reported to the user. This is an
 * error, it should not be used for warnings.
 *
 * @author Clément Fournier
 * @ingroup Validation
 */
public class InvalidLfSourceException extends RuntimeException {

  private final EObject node;
  private final String problem;

  public InvalidLfSourceException(EObject node, String problem) {
    super(problem);
    this.node = node;
    this.problem = problem;
  }

  public InvalidLfSourceException(String problem, EObject node) {
    this(node, problem);
  }

  public EObject getNode() {
    return node;
  }

  public String getProblem() {
    return problem;
  }
}
