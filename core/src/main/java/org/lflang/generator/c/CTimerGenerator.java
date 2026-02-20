package org.lflang.generator.c;

import java.util.List;
import org.lflang.TimeValue;
import org.lflang.generator.TimerInstance;
import org.lflang.lf.Expression;
import org.lflang.lf.ParameterReference;

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
    var selfRef = CUtil.reactorRef(timer.getParent());
    var offset = getTimerExpr(timer.getDefinition().getOffset(), selfRef, timer.getOffset());
    var period = getTimerExpr(timer.getDefinition().getPeriod(), selfRef, timer.getPeriod());
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

  /**
   * Return a C expression for a timer offset or period. If the AST expression
   * is a parameter reference, return a reference to the parameter field on
   * the self struct so that runtime overrides take effect. Otherwise, return
   * the resolved literal time expression.
   */
  private static String getTimerExpr(Expression expr, String selfRef, TimeValue resolved) {
    if (expr instanceof ParameterReference paramRef) {
      return selfRef + "->" + paramRef.getParameter().getName();
    }
    return CTypes.getInstance().getTargetTimeExpr(resolved);
  }
}
