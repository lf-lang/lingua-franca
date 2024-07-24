/**
 * @file
 * @author Benjamin Asch
 * @author Edward A. Lee
 * @copyright (c) 2023, The University of California at Berkeley License in <a
 *     href="https://github.com/lf-lang/lingua-franca/blob/master/LICENSE">BSD 2-clause</a>
 * @brief Instance of a watchdog
 */
package org.lflang.generator;

import java.util.LinkedHashSet;
import java.util.Set;
import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;

/**
 * Instance of a watchdog. Upon creation the actual delay is converted into a proper time value. If
 * a parameter is referenced, it is looked up in the given (grand)parent reactor instance.
 *
 * @author Benjamin Asch
 */
public class WatchdogInstance extends TriggerInstance<Watchdog> {

  /** Create a new watchdog instance associated with the given reactor instance. */
  public WatchdogInstance(Watchdog definition, ReactorInstance reactor) {
    super(definition, reactor);
    if (definition.getTimeout() != null) {
      // Get the timeout value given in the watchdog declaration.
      this.timeout = reactor.getTimeValue(definition.getTimeout());
    } else {
      // NOTE: The grammar does not allow the timeout to be omitted, so this should not occur.
      this.timeout = TimeValue.ZERO;
    }

    this.name = definition.getName();
    this.definition = definition;
    this.reactor = reactor;
    for (VarRef effect : definition.getEffects()) {
      Variable variable = effect.getVariable();
      if (variable instanceof Action) {
        // Effect is an Action.
        var actionInstance = reactor.lookupActionInstance((Action) variable);
        if (actionInstance != null) this.effects.add(actionInstance);
      }
      // Otherwise, do nothing (effect is either a mode or an unresolved reference).
    }
  }

  //////////////////////////////////////////////////////
  //// Public methods.

  public String getName() {
    return this.name;
  }

  public Watchdog getDefinition() {
    return this.definition;
  }

  public TimeValue getTimeout() {
    return this.timeout;
  }

  public ReactorInstance getReactor() {
    return this.reactor;
  }

  @Override
  public String toString() {
    return "WatchdogInstance " + name + "(" + timeout.toString() + ")";
  }

  //////////////////////////////////////////////////////
  //// Public fields.

  /** The ports or actions that this reaction may write to. */
  public Set<TriggerInstance<? extends Variable>> effects = new LinkedHashSet<>();

  //////////////////////////////////////////////////////
  //// Private fields.

  private final TimeValue timeout;

  private final String name;

  private final Watchdog definition;

  private final ReactorInstance reactor;
}
