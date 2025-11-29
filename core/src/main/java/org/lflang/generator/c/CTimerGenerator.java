package org.lflang.generator.c;

import java.util.List;
import org.lflang.generator.TimerInstance;

/**
 * Generates C code to declare and initialize timers.
 *
 * @author Edward A. Lee
 * @author {Soroush Bateni
 * @ingroup Generator
 */
public class CTimerGenerator {
  /**
   * Generate code to initialize the given timer.
   *
   * @param timer The timer to initialize for.
   * @param enc The enclave instance.
   */
  public static String generateInitializer(TimerInstance timer, CEnclaveInstance enc) {
    var triggerStructName = CUtil.reactorRef(timer.getParent()) + "->_lf__" + timer.getName();
    var offset = CTypes.getInstance().getTargetTimeExpr(timer.getOffset());
    var period = CTypes.getInstance().getTargetTimeExpr(timer.getPeriod());
    var mode = timer.getMode(false);
    var envId = enc.getReactorInstance().uniqueID();
    var modeRef =
        mode != null
            ? "&"
                + CUtil.reactorRef(mode.getParent())
                + "->_lf__modes["
                + mode.getParent().modes.indexOf(mode)
                + "];"
            : "NULL";

    return String.join(
        "\n",
        List.of(
            "// Initiaizing timer " + timer.getFullName() + ".",
            triggerStructName + ".offset = " + offset + ";",
            triggerStructName + ".period = " + period + ";",
            "// Associate timer with the environment of its parent",
            CUtil.ENVIRONMENT_VARIABLE_NAME
                + "["
                + envId
                + "].timer_triggers[timer_triggers_count["
                + envId
                + "]++] = &"
                + triggerStructName
                + ";",
            triggerStructName + ".mode = " + modeRef + ";"));
  }
}
