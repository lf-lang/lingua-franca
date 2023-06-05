package org.lflang.generator;

import org.lflang.lf.Action;
import org.lflang.lf.Reaction;
import org.lflang.lf.VarRef;

public interface DelayBodyGenerator {

  /** Constant that specifies how to name generated delay reactors. */
  String GEN_DELAY_CLASS_NAME = "_lf_GenDelay";

  /**
   * Generate code for the body of a reaction that takes an input and schedules an action with the
   * value of that input.
   *
   * @param action the action to schedule
   * @param port the port to read from
   */
  String generateDelayBody(Action action, VarRef port);

  /**
   * Generate code for the body of a reaction that is triggered by the given action and writes its
   * value to the given port.
   *
   * @param action the action that triggers the reaction
   * @param port the port to write to
   */
  String generateForwardBody(Action action, VarRef port);

  /**
   * Generate code for the generic type to be used in the class definition of a generated delay
   * reactor.
   */
  String generateDelayGeneric();

  /**
   * Indicates whether delay banks generated from after delays should have a variable length width.
   *
   * <p>If this is true, any delay reactors that are inserted for after delays on multiport
   * connections will have an unspecified variable length width. The code generator is then
   * responsible for inferring the correct width of the delay bank, which is only possible if the
   * precise connection width is known at compile time.
   *
   * <p>If this is false, the width specification of the generated bank will list all the ports
   * listed on the right side of the connection. This gives the code generator the information
   * needed to infer the correct width at runtime.
   */
  boolean generateAfterDelaysWithVariableWidth();

  /** Used to optionally apply additional transformations to the generated reactions */
  default void finalizeReactions(Reaction delayReaction, Reaction forwardReaction) {}
}
