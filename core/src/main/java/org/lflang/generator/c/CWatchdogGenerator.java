package org.lflang.generator.c;

import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;
import org.lflang.util.StringUtil;

/**
 * Generate C code for watchdogs. This class contains a collection of static methods supporting code
 * generation in C for watchdogs. These methods are protected because they are intended to be used
 * only within the same package.
 *
 * @author Benjamin Asch
 * @author Edward A. Lee
 * @ingroup Generator
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
    return !watchdogs.isEmpty();
  }

  /////////////////////////////////////////////////////////////////
  // Protected methods

  /**
   * For the specified reactor instance, generate initialization code for each watchdog in the
   * reactor. This code initializes the watchdog-related fields on the self struct of the reactor
   * instance. It also increments the watchdog count in the environment the parent reactor instance
   * is within.
   *
   * @param code The place to put the code
   * @param instance The reactor instance
   */
  protected static void generateInitializeWatchdogs(CodeBuilder code, ReactorInstance instance) {
    var foundOne = false;
    var temp = new CodeBuilder();
    var reactorRef = CUtil.reactorRef(instance);
    int watchdogCount = 0;
    var enclaveInfo = instance.containingEnclave;
    var enclaveStruct = CUtil.getEnvironmentStruct(instance.containingEnclave);
    var enclaveId = instance.containingEnclaveReactor.uniqueID();

    for (Watchdog watchdog :
        ASTUtils.allWatchdogs(ASTUtils.toDefinition(instance.getDefinition().getReactorClass()))) {
      var watchdogField = reactorRef + "->_lf_watchdog_" + watchdog.getName();
      temp.pr(
          String.join(
              "\n",
              enclaveStruct
                  + ".watchdogs[watchdog_count["
                  + enclaveId
                  + "]++] = &"
                  + watchdogField
                  + ";",
              watchdogField
                  + ".min_expiration = "
                  + CTypes.getInstance()
                      .getTargetTimeExpr(instance.getTimeValue(watchdog.getTimeout()))
                  + ";",
              watchdogField + ".active = false;",
              watchdogField + ".terminate = false;",
              "if (" + watchdogField + ".base->reactor_mutex == NULL) {",
              "   "
                  + watchdogField
                  + ".base->reactor_mutex = (lf_mutex_t*)calloc(1, sizeof(lf_mutex_t));",
              "}"));
      watchdogCount += 1;
      foundOne = true;
    }
    if (foundOne) {
      code.pr(temp.toString());
    }
    enclaveInfo.numWatchdogs += watchdogCount;
  }

  /**
   * Generate watchdog functions definition for a reactor. These functions have a single argument
   * that is a void* pointing to the self struct of the reactor, which contains parameters, state
   * variables, inputs (triggering or not), actions (triggering or produced), and outputs.
   *
   * @param src The place to put the code
   * @param header The place to put header code
   * @param tpr The reactor declaration
   * @param suppressLineDirectives Whether to suppress the generation of line directives.
   * @param messageReporter Used to report errors and warnings.
   */
  protected static void generateWatchdogs(
      CodeBuilder src,
      CodeBuilder header,
      TypeParameterizedReactor tpr,
      boolean suppressLineDirectives,
      MessageReporter messageReporter) {
    if (hasWatchdogs(tpr.reactor())) {
      header.pr("#include \"core/threaded/watchdog.h\"");
      for (Watchdog watchdog : ASTUtils.allWatchdogs(tpr.reactor())) {
        src.pr(generateWatchdogFunction(watchdog, tpr, messageReporter, suppressLineDirectives));
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

      body.pr("watchdog_t _lf_watchdog_" + watchdogName + ";");

      // watchdog function name
      var watchdogFunctionName = watchdogFunctionName(watchdog, tpr);
      // Set values of watchdog_t struct in the reactor's constructor.
      constructorCode.pr(
          String.join(
              "\n",
              "self->_lf_watchdog_" + watchdogName + ".base = &(self->base);",
              "self->_lf_watchdog_" + watchdogName + ".expiration = NEVER;",
              "self->_lf_watchdog_" + watchdogName + ".active = false;",
              "self->_lf_watchdog_" + watchdogName + ".terminate = false;",
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
    code.pr(
        structType + "* self = (" + structType + "*)instance_args; SUPPRESS_UNUSED_WARNING(self);");

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
        } else if (variable instanceof Action) {
          watchdogInitialization.pr(generateActionVariablesInHandler((Action) variable, tpr));
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
   * Generate action variables for the watchdog handler.
   *
   * @param action The action.
   */
  private static String generateActionVariablesInHandler(
      Action action, TypeParameterizedReactor tpr) {
    String structType = CGenerator.variableStructType(action, tpr, false);
    CodeBuilder builder = new CodeBuilder();
    builder.pr(
        "// Expose the action struct as a local variable whose name matches the action name.");
    builder.pr(structType + "* " + action.getName() + " = &self->_lf_" + action.getName() + ";");
    return builder.toString();
  }

  /**
   * Do heavy lifting to generate the watchdog handler function
   *
   * @param header function name and declaration.
   * @param init initialize variable.
   * @param watchdog The watchdog.
   * @param suppressLineDirectives Whether to suppress the generation of line directives.
   */
  private static String generateFunction(
      String header, String init, Watchdog watchdog, boolean suppressLineDirectives) {
    var function = new CodeBuilder();
    function.pr("#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetHeader()));
    function.pr(
        """
        #ifdef __cplusplus
        extern "C" {
        #endif
        #include "reactor_common.h"
        #ifdef __cplusplus
        }
        #endif
        """);
    function.pr(header + " {");
    function.indent();
    function.pr(init);
    function.pr("{"); // Limit scope.
    function.indent();
    function.pr("environment_t * __env = self->base.environment;");
    function.pr("LF_MUTEX_LOCK(&__env->mutex);");
    function.pr("tag_t tag = {.time =" + watchdog.getName() + "->expiration , .microstep=0};");
    function.pr("if (lf_tag_compare(tag, lf_tag()) <= 0) { ");
    function.indent();
    function.pr("tag = lf_tag();");
    function.pr("tag.microstep++;");
    function.unindent();
    function.pr("}");
    function.pr("_lf_schedule_at_tag(__env, " + watchdog.getName() + "->trigger, tag, NULL);");
    function.pr("lf_cond_broadcast(&__env->event_q_changed);");
    function.pr("LF_MUTEX_UNLOCK(&__env->mutex);");
    function.unindent();
    function.pr("}");
    function.prSourceLineNumber(watchdog.getCode(), suppressLineDirectives);
    function.pr(ASTUtils.toText(watchdog.getCode()));
    function.prEndSourceLineNumber(suppressLineDirectives);
    function.unindent();
    function.pr("}");
    function.pr(
        "#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetUndefHeader()));
    return function.toString();
  }

  /** Generate the watchdog handler function. */
  private static String generateWatchdogFunction(
      Watchdog watchdog,
      TypeParameterizedReactor tpr,
      MessageReporter messageReporter,
      boolean suppressLineDirectives) {
    return generateFunction(
        generateWatchdogFunctionHeader(watchdog, tpr),
        generateInitializationForWatchdog(watchdog, tpr, messageReporter),
        watchdog,
        suppressLineDirectives);
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
