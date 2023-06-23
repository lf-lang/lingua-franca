/**
 * @file
 * @author Benjamin Asch
 * @author Edward A. Lee
 * @copyright (c) 2023, The University of California at Berkeley. License: <a
 *     href="https://github.com/lf-lang/lingua-franca/blob/master/LICENSE">BSD 2-clause</a>
 * @brief Code generation methods for watchdogs in C.
 */
package org.lflang.generator.c;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;

/**
 * @brief Generate C code for watchdogs. This class contains a collection of static methods
 *     supporting code generation in C for watchdogs. These methods are protected because they are
 *     intended to be used only within the same package.
 * @author Benjamin Asch
 * @author Edward A. Lee
 */
public class CWatchdogGenerator {

  /**
   * Return true if the given reactor has one or more watchdogs.
   *
   * @param reactor The reactor.
   * @return True if the given reactor has watchdogs.
   */
  public static boolean hasWatchdogs(Reactor reactor) {
    List<Watchdog> watchdogs = ASTUtils.allWatchdogs(reactor);
    if (watchdogs != null && !watchdogs.isEmpty()) return true;
    return false;
  }

  /////////////////////////////////////////////////////////////////
  // Protected methods

  /**
   * For the specified reactor instance, generate initialization code for each watchdog in the
   * reactor. This code initializes the watchdog-related fields on the self struct of the reactor
   * instance.
   *
   * @param code The place to put the code
   * @param instance The reactor instance
   * @return The count of watchdogs found in the reactor
   */
  protected static int generateInitializeWatchdogs(CodeBuilder code, ReactorInstance instance) {
    var foundOne = false;
    var temp = new CodeBuilder();
    var reactorRef = CUtil.reactorRef(instance);
    int watchdogCount = 0;
    for (Watchdog watchdog :
        ASTUtils.allWatchdogs(ASTUtils.toDefinition(instance.getDefinition().getReactorClass()))) {
      var watchdogField = reactorRef + "->_lf_watchdog_" + watchdog.getName();
      temp.pr(
          String.join(
              "\n",
              "_lf_watchdogs[watchdog_number++] = &" + watchdogField + ";",
              watchdogField
                  + ".min_expiration = "
                  + CTypes.getInstance()
                      .getTargetTimeExpr(instance.getTimeValue(watchdog.getTimeout()))
                  + ";",
              watchdogField + ".thread_active = false;",
              "if (" + watchdogField + ".base->reactor_mutex == NULL) {",
              "   "
                  + watchdogField
                  + ".base->reactor_mutex = (lf_mutex_t*)calloc(1, sizeof(lf_mutex_t));",
              "}"));
      watchdogCount += 1;
      foundOne = true;
    }
    // temp.pr("#endif");
    if (foundOne) {
      code.pr(temp.toString());
    }
    code.pr("SUPPRESS_UNUSED_WARNING(_lf_watchdog_count);");
    return watchdogCount;
  }

  /**
   * Generate watchdog functions definition for a reactor. These functions have a single argument
   * that is a void* pointing to the self struct of the reactor, which contains parameters, state
   * variables, inputs (triggering or not), actions (triggering or produced), and outputs.
   *
   * @param src The place to put the code
   * @param header The place to put header code
   * @param tpr The reactor declaration
   */
  protected static void generateWatchdogs(
      CodeBuilder src,
      CodeBuilder header,
      TypeParameterizedReactor tpr,
      MessageReporter messageReporter) {
    if (hasWatchdogs(tpr.reactor())) {
      header.pr("#include \"core/threaded/watchdog.h\"");
      for (Watchdog watchdog : ASTUtils.allWatchdogs(tpr.reactor())) {
        src.pr(generateWatchdogFunction(watchdog, tpr, messageReporter));
      }
    }
  }

  /**
   * Generate watchdog definitions in the reactor's self struct.
   *
   * @param body The place to put the definitions
   * @param tpr The concrete reactor class
   * @param constructorCode The place to put initialization code.
   */
  protected static void generateWatchdogStruct(
      CodeBuilder body, TypeParameterizedReactor tpr, CodeBuilder constructorCode) {

    for (Watchdog watchdog : ASTUtils.allWatchdogs(tpr.reactor())) {
      String watchdogName = watchdog.getName();

      body.pr(watchdog, "watchdog_t _lf_watchdog_" + watchdogName + ";");

      // watchdog function name
      var watchdogFunctionName = watchdogFunctionName(watchdog, tpr);
      // Set values of watchdog_t struct in the reactor's constructor.
      constructorCode.pr(
          watchdog,
          String.join(
              "\n",
              "self->_lf_watchdog_" + watchdogName + ".base = &(self->base);",
              "self->_lf_watchdog_" + watchdogName + ".expiration = NEVER;",
              "self->_lf_watchdog_" + watchdogName + ".thread_active = false;",
              "self->_lf_watchdog_"
                  + watchdogName
                  + ".watchdog_function = "
                  + watchdogFunctionName
                  + ";",
              "self->_lf_watchdog_"
                  + watchdogName
                  + ".trigger = &(self->_lf__"
                  + watchdogName
                  + ");"));
    }
  }

  /**
   * Generate a global table of watchdog structs.
   *
   * @param count The number of watchdogs found.
   * @return The code that defines the table or a comment if count is 0.
   */
  protected static String generateWatchdogTable(int count) {
    if (count == 0) {
      return String.join(
          "\n",
          "// No watchdogs found.",
          "typedef void watchdog_t;",
          "watchdog_t* _lf_watchdogs = NULL;",
          "int _lf_watchdog_count = 0;");
    }
    return String.join(
        "\n",
        List.of(
            "// Array of pointers to watchdog structs.",
            "watchdog_t* _lf_watchdogs[" + count + "];",
            "int _lf_watchdog_count = " + count + ";"));
  }

  /////////////////////////////////////////////////////////////////
  // Private methods

  /**
   * Generate necessary initialization code inside the body of a watchdog handler.
   *
   * @param watchdog The wotchdog
   * @param tpr The concrete reactor class that has the watchdog
   */
  private static String generateInitializationForWatchdog(
      Watchdog watchdog, TypeParameterizedReactor tpr, MessageReporter messageReporter) {

    // Construct the reactionInitialization code to go into
    // the body of the function before the verbatim code.
    CodeBuilder watchdogInitialization = new CodeBuilder();

    CodeBuilder code = new CodeBuilder();

    // Define the "self" struct.
    String structType = CUtil.selfType(tpr);
    // A null structType means there are no inputs, state,
    // or anything else. No need to declare it.
    if (structType != null) {
      code.pr(
          String.join(
              "\n",
              structType
                  + "* self = ("
                  + structType
                  + "*)instance_args; SUPPRESS_UNUSED_WARNING(self);"));
    }

    // Declare mode if in effects field of watchdog
    if (watchdog.getEffects() != null) {
      for (VarRef effect : watchdog.getEffects()) {
        Variable variable = effect.getVariable();
        if (variable instanceof Mode) {
          // Mode change effect
          int idx = ASTUtils.allModes(tpr.reactor()).indexOf((Mode) effect.getVariable());
          String name = effect.getVariable().getName();
          if (idx >= 0) {
            watchdogInitialization.pr(
                "reactor_mode_t* "
                    + name
                    + " = &self->_lf__modes["
                    + idx
                    + "];\n"
                    + "lf_mode_change_type_t _lf_"
                    + name
                    + "_change_type = "
                    + (effect.getTransition() == ModeTransition.HISTORY
                        ? "history_transition"
                        : "reset_transition")
                    + ";");
          } else {
            messageReporter
                .at(watchdog)
                .error(
                    "In generateInitializationForWatchdog(): "
                        + name
                        + " not a valid mode of this reactor.");
          }
        }
      }
    }
    // Add watchdog definition
    watchdogInitialization.pr(
        "watchdog_t* "
            + watchdog.getName()
            + " = &(self->_lf_watchdog_"
            + watchdog.getName()
            + ");\n");

    // Next generate all the collected setup code.
    code.pr(watchdogInitialization.toString());
    return code.toString();
  }

  /**
   * Do heavy lifting to generate the watchdog handler function
   *
   * @param header function name and declaration.
   * @param init initialize variable.
   * @param watchdog The watchdog.
   */
  private static String generateFunction(String header, String init, Watchdog watchdog) {
    var function = new CodeBuilder();
    function.pr(header + " {");
    function.indent();
    function.pr(init);
    function.pr(
        "_lf_schedule(self->base.environment, (*" + watchdog.getName() + ").trigger, 0, NULL);");
    function.prSourceLineNumber(watchdog.getCode());
    function.pr(ASTUtils.toText(watchdog.getCode()));
    function.unindent();
    function.pr("}");
    return function.toString();
  }

  /** Generate the watchdog handler function. */
  private static String generateWatchdogFunction(
      Watchdog watchdog, TypeParameterizedReactor tpr, MessageReporter messageReporter) {
    return generateFunction(
        generateWatchdogFunctionHeader(watchdog, tpr),
        generateInitializationForWatchdog(watchdog, tpr, messageReporter),
        watchdog);
  }

  /**
   * Return the start of a C function definition for a watchdog.
   *
   * @param watchdog The watchdog
   * @param tpr The concrete reactor class
   * @return The function name for the watchdog function.
   */
  private static String generateWatchdogFunctionHeader(
      Watchdog watchdog, TypeParameterizedReactor tpr) {
    String functionName = watchdogFunctionName(watchdog, tpr);
    return CReactionGenerator.generateFunctionHeader(functionName);
  }

  /**
   * Return the name of the watchdog expiration handler function.
   *
   * @param watchdog The watchdog
   * @param tpr The concrete reactor class that has the watchdog
   * @return Name of the watchdog handler function
   */
  private static String watchdogFunctionName(Watchdog watchdog, TypeParameterizedReactor tpr) {
    return tpr.getName().toLowerCase()
        + "_"
        + watchdog.getName().toLowerCase()
        + "_watchdog_function";
  }
}
