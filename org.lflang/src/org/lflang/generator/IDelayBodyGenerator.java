package org.lflang.generator;

import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

public interface IDelayBodyGenerator {

    /**
     * Constant that specifies how to name generated delay reactors.
     */
    String GEN_DELAY_CLASS_NAME = "_lf_GenDelay";

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     *
     * @param action the action to schedule
     * @param port the port to read from
     */
    String generateDelayBody(Action action, VarRef port);

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     *
     * @param action the action that triggers the reaction
     * @param port the port to write to
     */
    String generateForwardBody(Action action, VarRef port);

    /**
     * Generate code for the generic type to be used in the class definition
     * of a generated delay reactor.
     */
    String generateDelayGeneric();

}
