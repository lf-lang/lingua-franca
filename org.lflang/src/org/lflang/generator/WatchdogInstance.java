/**
 * @file
 * @author Benjamin Asch
 * @author Edward A. Lee
 * @copyright (c) 2023, The University of California at Berkeley
 * License in [BSD 2-clause](https://github.com/lf-lang/lingua-franca/blob/master/LICENSE)
 * @brief Instance of a watchdog
 */
package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Watchdog;

/**
 * Instance of a watchdog. Upon creation the actual delay is converted into a proper time value. If
 * a parameter is referenced, it is looked up in the given (grand)parent reactor instance.
 *
 * @author{Benjamin Asch <benjamintasch@berkeley.edu>}
 */
public class WatchdogInstance {

  /** Create a new watchdog instance associated with the given reactor instance. */
  public WatchdogInstance(Watchdog definition, ReactorInstance reactor) {
    if (definition.getTimeout() != null) {
      // Get the timeout value given in the watchdog declaration.
      this.timeout = reactor.getTimeValue(definition.getTimeout());
    } else {
      // NOTE: The grammar does not allow the timeout to be omitted, so this should not occur.
      this.timeout = TimeValue.ZERO;
    }

    this.name = definition.getName().toString();
    this.definition = definition;
    this.reactor = reactor;
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
    return (TimeValue) this.timeout;
  }

  public ReactorInstance getReactor() {
    return this.reactor;
  }

  @Override
  public String toString() {
    return "WatchdogInstance " + name + "(" + timeout.toString() + ")";
  }

  //////////////////////////////////////////////////////
  //// Private fields.

  private final TimeValue timeout;

  private final String name;

  private final Watchdog definition;

  private final ReactorInstance reactor;
}
