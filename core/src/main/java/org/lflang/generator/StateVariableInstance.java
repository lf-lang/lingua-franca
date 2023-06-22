/** A data structure for a state variable. */

/*************
 * Copyright (c) 2019-2022, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.generator;

import org.lflang.MessageReporter;
import org.lflang.lf.StateVar;

/** Representation of a compile-time instance of a state variable. */
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
