package org.lflang.generator;

import java.util.List;
import java.util.Optional;
import org.lflang.InferredType;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Assignment;
import org.lflang.lf.Initializer;
import org.lflang.lf.Parameter;

/**
 * Representation of a compile-time instance of a parameter. Upon creation, it is checked whether
 * this parameter is overridden by an assignment in the instantiation that this parameter instance
 * is a result of. If it is overridden, the parameter gets initialized using the value looked up in
 * the instantiation hierarchy.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @ingroup Instances
 */
public class ParameterInstance extends NamedInstance<Parameter> {

  /**
   * Create a runtime instance from the specified definition and with the specified parent that
   * instantiated it.
   *
   * @param definition The declaration in the AST.
   * @param parent The reactor instance this parameter is a part of.
   */
  public ParameterInstance(Parameter definition, ReactorInstance parent) {
    super(definition, parent);
    if (parent == null) {
      throw new InvalidSourceException("Cannot create a ParameterInstance with no parent.");
    }

    this.type = ASTUtils.getInferredType(definition);
  }

  /////////////////////////////////////////////
  //// Public Fields

  public InferredType type;

  /////////////////////////////////////////////
  //// Public Methods

  /** Get the initial value of this parameter. */
  private Initializer getInitialValue() {
    return definition.getInit();
  }

  /**
   * Return the (possibly overridden) value of this parameter in the containing instance. Parameter
   * references are resolved to actual expressions.
   */
  public Initializer getActualValue() {
    Assignment override = getOverride();
    Initializer init;
    if (override != null) {
      init = override.getRhs();
    } else {
      init = getInitialValue();
    }
    return init;
  }

  /**
   * Return the name of this parameter.
   *
   * @return The name of this parameter.
   */
  public String getName() {
    return this.definition.getName();
  }

  /**
   * Return the assignment that overrides this parameter in the parent's instantiation or null if
   * there is no override.
   */
  public Assignment getOverride() {
    List<Assignment> assignments = parent.definition.getParameters();
    Optional<Assignment> assignment =
        assignments.stream().filter(it -> it.getLhs() == definition).findFirst();
    return assignment.orElse(null);
  }

  /** Return a descriptive string. */
  @Override
  public String toString() {
    return "ParameterInstance " + getFullName();
  }
}
