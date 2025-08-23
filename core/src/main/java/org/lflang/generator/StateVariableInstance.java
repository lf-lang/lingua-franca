package org.lflang.generator;

import org.lflang.MessageReporter;
import org.lflang.lf.StateVar;

/**
 * Representation of a compile-time instance of a state variable.
 *
 * @ingroup Instances
 */
public class StateVariableInstance extends NamedInstance<StateVar> {

  /**
   * Create a runtime instance from the specified definition and with the specified parent that
   * instantiated it.
   *
   * @param definition The declaration in the AST.
   * @param parent The parent.
   */
  public StateVariableInstance(StateVar definition, ReactorInstance parent) {
    this(definition, parent, null);
  }

  /**
   * Create a port instance from the specified definition and with the specified parent that
   * instantiated it.
   *
   * @param definition The declaration in the AST.
   * @param parent The parent.
   * @param errorReporter An error reporter, or null to throw exceptions.
   */
  public StateVariableInstance(
      StateVar definition, ReactorInstance parent, MessageReporter errorReporter) {
    super(definition, parent);

    if (parent == null) {
      throw new NullPointerException("Cannot create a StateVariableInstance with no parent.");
    }
  }

  //////////////////////////////////////////////////////
  //// Public methods

  /**
   * Return the name of this trigger.
   *
   * @return The name of this trigger.
   */
  @Override
  public String getName() {
    return definition.getName();
  }

  @Override
  public String toString() {
    return "StateVariableInstance " + getFullName();
  }
}
