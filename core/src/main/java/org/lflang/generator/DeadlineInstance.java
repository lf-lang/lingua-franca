/** A data structure for a deadline instance. */

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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Deadline;

/**
 * Instance of a deadline. Upon creation the actual delay is converted into a proper time value. If
 * a parameter is referenced, it is looked up in the given (grand)parent reactor instance.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 */
public class DeadlineInstance {

  /** Create a new deadline instance associated with the given reaction instance. */
  public DeadlineInstance(Deadline definition, ReactionInstance reaction) {
    if (definition.getDelay() != null) {
      this.maxDelay = reaction.parent.getTimeValue(definition.getDelay());
    } else {
      this.maxDelay = TimeValue.ZERO;
    }
  }

  //////////////////////////////////////////////////////
  //// Public fields.

  /**
   * The delay D associated with this deadline. If physical time T < logical time t + D, the
   * deadline is met, otherwise, it is violated.
   */
  public final TimeValue maxDelay;

  //////////////////////////////////////////////////////
  //// Public methods.

  @Override
  public String toString() {
    return "DeadlineInstance " + maxDelay.toString();
  }
}
