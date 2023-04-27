package org.lflang.generator.c;

import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;

/**
 * Generates necessary C code for watchdogs.
 *
 * @author{Benjamin Asch <benjamintasch@berkeley.edu>}
 */
public class CWatchdogGenerator {

  /**
   * Generate necessary initialization code inside the body of the watchdog that belongs to reactor
   * decl.
   *
   * @param decl The reactor that has the watchdog
   */
  public static String generateInitializationForWatchdog(Watchdog watchdog, ReactorDecl decl) {
    Reactor reactor = ASTUtils.toDefinition(decl);

    // Construct the reactionInitialization code to go into
    // the body of the function before the verbatim code.
    CodeBuilder watchdogInitialization = new CodeBuilder();

    CodeBuilder code = new CodeBuilder();

    // Define the "self" struct.
    String structType = CUtil.selfType(reactor);
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
          int idx = ASTUtils.allModes(reactor).indexOf((Mode) effect.getVariable());
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
          }
          // FIXME: include error reporter
          // else {
          //     errorReporter.reportError(
          //         watchdog,
          //         "In generateWatchdog(): " + name + " not a valid mode of this reactor."
          //     );
          // }
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
   * Returns the name of the watchdog function for reaction.
   *
   * @param decl The reactor with the watchdog
   * @param watchdog The watchdog
   * @return Name of the watchdog function for reaction
   */
  public static String generateWatchdogFunctionName(Watchdog watchdog, ReactorDecl decl) {
    return decl.getName().toLowerCase()
        + "_"
        + watchdog.getName().toLowerCase()
        + "_watchdog_function";
  }

  /**
   * Return the top level C function header for the watchdog function in "decl"
   *
   * @param decl The reactor declaration
   * @param watchdog The watchdog.
   * @return The function name for the watchdog function.
   */
  public static String generateWatchdogFunctionHeader(Watchdog watchdog, ReactorDecl decl) {
    String functionName = generateWatchdogFunctionName(watchdog, decl);
    return CReactionGenerator.generateFunctionHeader(functionName);
  }

  /** Generate the watchdog function. */
  public static String generateWatchdogFunction(Watchdog watchdog, ReactorDecl decl) {
    return generateFunction(
        generateWatchdogFunctionHeader(watchdog, decl),
        generateInitializationForWatchdog(watchdog, decl),
        watchdog);
  }

  /**
   * Do heavy lifting to generate above watchdog function
   *
   * @param header function name and declaration.
   * @param init initialize variable.
   * @param watchdog The watchdog.
   */
  public static String generateFunction(String header, String init, Watchdog watchdog) {
    var function = new CodeBuilder();
    function.pr(header + " {");
    function.indent();
    function.pr(init);
    function.pr("_lf_schedule((*" + watchdog.getName() + ").trigger, 0, NULL);");
    function.prSourceLineNumber(watchdog.getCode());
    function.pr(ASTUtils.toText(watchdog.getCode()));
    function.unindent();
    function.pr("}");
    return function.toString();
  }

  /** Generate watchdog definition in parent struct. */
  public static void generateWatchdogStruct(
      CodeBuilder body, ReactorDecl decl, CodeBuilder constructorCode) {
    var reactor = ASTUtils.toDefinition(decl);

    for (Watchdog watchdog : ASTUtils.allWatchdogs(reactor)) {
      String watchdogName = watchdog.getName();

      body.pr(watchdog, "watchdog_t _lf_watchdog_" + watchdogName + ";");

      // watchdog function name
      var watchdogFunctionName = generateWatchdogFunctionName(watchdog, decl);
      // Set values of watchdog_t struct in the reactor's constructor
      // FIXME: update parameters
      // constructorCode.pr("#ifdef LF_THREADED");
      constructorCode.pr(
          watchdog,
          String.join(
              "\n",
              "self->_lf_watchdog_" + watchdogName + ".base = &(self->base);",
              "self->_lf_watchdog_" + watchdogName + ".expiration = NEVER;",
              "self->_lf_watchdog_" + watchdogName + ".thread_active = false;",
              // "self->_lf_watchdog_"+watchdogName+".min_expiration = "+min_expiration+";",
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
   * Generate a watchdog function definition for a reactor. This function will have a single
   * argument that is a void* pointing to a struct that contains parameters, state variables, inputs
   * (triggering or not), actions (triggering or produced), and outputs.
   *
   * @param watchdog The watchdog.
   * @param decl The reactor.
   */
  public static String generateWatchdog(Watchdog watchdog, ReactorDecl decl) {
    var code = new CodeBuilder();

    code.pr(generateWatchdogFunction(watchdog, decl));

    return code.toString();
  }

  public static String generateBuiltinTriggersTable(int count, String name) {
    return String.join(
        "\n",
        List.of(
            "// Array of pointers to " + name + " triggers.",
            "#ifdef LF_THREADED",
            (count > 0
                    ? "   watchdog_t* _lf_" + name + "s[" + count + "]"
                    : "   watchdog_t* _lf_" + name + "s = NULL")
                + ";",
            "   int _lf_" + name + "_number = " + count + ";",
            "#endif"));
  }
}
