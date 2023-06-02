/** Instance of a trigger (port, action, or timer). */

/*************
 * Copyright (c) 2019, The University of California at Berkeley.
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

import java.util.LinkedHashSet;
import java.util.Set;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.Variable;
import org.lflang.lf.impl.VariableImpl;

/**
 * Instance of a trigger (port, action, or timer).
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 * @author Alexander Schulz-Rosengarten
 */
public class TriggerInstance<T extends Variable> extends NamedInstance<T> {

  /**
   * Construct a new instance with the specified definition and parent. E.g., for a action instance,
   * the definition is Action, and for a port instance, it is Port. These are nodes in the AST. This
   * is protected because only subclasses should be constructed.
   *
   * @param definition The definition in the AST for this instance.
   * @param parent The reactor instance that creates this instance.
   */
  protected TriggerInstance(T definition, ReactorInstance parent) {
    super(definition, parent);
  }

  /**
   * Construct a new instance for a special builtin trigger.
   *
   * @param trigger The actual trigger definition.
   * @param parent The reactor instance that creates this instance.
   */
  static TriggerInstance<BuiltinTriggerVariable> builtinTrigger(
      BuiltinTriggerRef trigger, ReactorInstance parent) {
    return new TriggerInstance<>(new BuiltinTriggerVariable(trigger), parent);
  }

  /////////////////////////////////////////////
  //// Public Methods

  /**
   * Return the reaction instances that are triggered or read by this trigger. If this port is an
   * output, then the reaction instances belong to the parent of the port's parent. If the port is
   * an input, then the reaction instances belong to the port's parent.
   */
  public Set<ReactionInstance> getDependentReactions() {
    return dependentReactions;
  }

  /**
   * Return the reaction instances that may send data via this port. If this port is an input, then
   * the reaction instance belongs to parent of the port's parent. If it is an output, the reaction
   * instance belongs to the port's parent.
   */
  public Set<ReactionInstance> getDependsOnReactions() {
    return dependsOnReactions;
  }

  /**
   * Return the name of this trigger.
   *
   * @return The name of this trigger.
   */
  @Override
  public String getName() {
    return definition.getName();
  }

  /** Return true if this trigger is "shutdown". */
  public boolean isShutdown() {
    return isBuiltInType(BuiltinTrigger.SHUTDOWN);
  }

  /** Return true if this trigger is "startup"./ */
  public boolean isStartup() {
    return isBuiltInType(BuiltinTrigger.STARTUP);
  }

  /** Return true if this trigger is "startup"./ */
  public boolean isReset() {
    return isBuiltInType(BuiltinTrigger.RESET);
  }

  /////////////////////////////////////////////
  //// Private Methods

  private boolean isBuiltInType(BuiltinTrigger type) {
    return this.definition instanceof BuiltinTriggerVariable
        && ((BuiltinTriggerRef) ((BuiltinTriggerVariable) this.definition).definition)
            .getType()
            .equals(type);
  }

  /////////////////////////////////////////////
  //// Protected Fields

  /**
   * Reaction instances that are triggered or read by this trigger. If this port is an output, then
   * the reaction instances belong to the parent of the port's parent. If the port is an input, then
   * the reaction instances belong to the port's parent.
   */
  Set<ReactionInstance> dependentReactions = new LinkedHashSet<>();

  /**
   * Reaction instances that may send data via this port. If this port is an input, then the
   * reaction instance belongs to parent of the port's parent. If it is an output, the reaction
   * instance belongs to the port's parent.
   */
  Set<ReactionInstance> dependsOnReactions = new LinkedHashSet<>();

  /////////////////////////////////////////////
  //// Special class for builtin triggers

  /** This class allows to have BuiltinTriggers represented by a Variable type. */
  public static class BuiltinTriggerVariable extends VariableImpl {

    /** The builtin trigger type represented by this variable. */
    public final BuiltinTrigger type;

    /** The actual TriggerRef definition in the AST. */
    public final TriggerRef definition;

    public BuiltinTriggerVariable(BuiltinTriggerRef trigger) {
      this.type = trigger.getType();
      this.definition = trigger;
    }

    @Override
    public String getName() {
      return this.type.name().toLowerCase();
    }

    @Override
    public void setName(String newName) {
      throw new UnsupportedOperationException(
          this.getClass().getName() + " has an immutable name.");
    }
  }
}
