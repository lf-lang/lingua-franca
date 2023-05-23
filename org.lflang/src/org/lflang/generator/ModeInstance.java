/*************
 * Copyright (c) 2021, Kiel University.
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

import java.util.LinkedList;
import java.util.List;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.VarRef;

/**
 * Representation of a runtime instance of a mode.
 *
 * @author Alexander Schulz-Rosengarten
 */
public class ModeInstance extends NamedInstance<Mode> {

  /**
   * Create a new reaction instance from the specified definition within the specified parent. This
   * constructor should be called only by the ReactorInstance class after all other contents
   * (reactions, etc.) are registered because this constructor call will look them up.
   *
   * @param definition A mode definition.
   * @param parent The parent reactor instance, which cannot be null.
   */
  protected ModeInstance(Mode definition, ReactorInstance parent) {
    super(definition, parent);

    collectMembers();
  }

  ////////////////////////////////////////////////////
  // Member fields.

  /** The action instances belonging to this mode instance. */
  public List<ActionInstance> actions = new LinkedList<ActionInstance>();

  /** The reactor instances belonging to this mode instance, in order of declaration. */
  public List<ReactorInstance> instantiations = new LinkedList<ReactorInstance>();

  /** List of reaction instances for this reactor instance. */
  public List<ReactionInstance> reactions = new LinkedList<ReactionInstance>();

  /** The timer instances belonging to this reactor instance. */
  public List<TimerInstance> timers = new LinkedList<TimerInstance>();

  /** The outgoing transitions of this mode. */
  public List<Transition> transitions = new LinkedList<Transition>();

  ////////////////////////////////////////////////////
  // Public methods.

  /**
   * Return the name of this mode.
   *
   * @return The name of this mode.
   */
  @Override
  public String getName() {
    return this.definition.getName();
  }

  /** {@inheritDoc} */
  @Override
  public ReactorInstance root() {
    return parent.root();
  }

  /** Return a descriptive string. */
  @Override
  public String toString() {
    return getName() + " of " + parent.getFullName();
  }

  /** Returns true iff this mode is the initial mode of this reactor instance. */
  public boolean isInitial() {
    return definition.isInitial();
  }

  /**
   * Sets up all transitions that leave this mode. Requires that all mode instances and other
   * contents (reactions, etc.) of the parent reactor are created.
   */
  public void setupTranstions() {
    transitions.clear();
    for (var reaction : reactions) {
      for (var effect : reaction.definition.getEffects()) {
        if (effect instanceof VarRef) {
          var target = effect.getVariable();
          if (target instanceof Mode) {
            transitions.add(
                new Transition(this, parent.lookupModeInstance((Mode) target), reaction, effect));
          }
        }
      }
    }
  }

  /** Returns true iff this mode contains the given instance. */
  public boolean contains(NamedInstance<?> instance) {
    if (instance instanceof TimerInstance) {
      return timers.contains(instance);
    } else if (instance instanceof ActionInstance) {
      return actions.contains(instance);
    } else if (instance instanceof ReactorInstance) {
      return instantiations.contains(instance);
    } else if (instance instanceof ReactionInstance) {
      return reactions.contains(instance);
    } else {
      return false;
    }
  }

  ////////////////////////////////////////////////////
  // Private methods.

  private void collectMembers() {
    // Collect timers
    for (var decl : definition.getTimers()) {
      var instance = parent.lookupTimerInstance(decl);
      if (instance != null) {
        this.timers.add(instance);
      }
    }

    // Collect actions
    for (var decl : definition.getActions()) {
      var instance = parent.lookupActionInstance(decl);
      if (instance != null) {
        this.actions.add(instance);
      }
    }

    // Collect reactor instantiation
    for (var decl : definition.getInstantiations()) {
      var instance = parent.lookupReactorInstance(decl);
      if (instance != null) {
        this.instantiations.add(instance);
      }
    }

    // Collect reactions
    for (var decl : definition.getReactions()) {
      var instance = parent.lookupReactionInstance(decl);
      if (instance != null) {
        this.reactions.add(instance);
      }
    }
  }

  ////////////////////////////////////////////////////
  // Data class.

  public static class Transition extends NamedInstance<VarRef> {
    public final ModeInstance source;
    public final ModeInstance target;
    public final ReactionInstance reaction;
    public final ModeTransition type;

    Transition(
        ModeInstance source, ModeInstance target, ReactionInstance reaction, VarRef definition) {
      super(definition, source.parent);
      this.source = source;
      this.target = target;
      this.reaction = reaction;
      this.type =
          definition.getTransition() == null ? ModeTransition.RESET : definition.getTransition();
    }

    @Override
    public String getName() {
      return this.source.getName() + " -> " + this.target + " by " + this.reaction.getName();
    }

    @Override
    public ReactorInstance root() {
      return this.parent.root();
    }

    public ModeTransition getType() {
      return type;
    }
  }
}
