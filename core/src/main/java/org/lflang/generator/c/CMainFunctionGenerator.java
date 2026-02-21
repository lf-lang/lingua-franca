package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Assignment;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Literal;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;
import org.lflang.lf.WidthSpec;
import org.lflang.lf.WidthTerm;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.type.PlatformType.Platform;
import org.lflang.util.StringUtil;

/**
 * Generate the code that is the entry point of the program.
 *
 * @ingroup Generator
 */
public class CMainFunctionGenerator {
  private TargetConfig targetConfig;

  /** The main reactor definition, or null if there is no main reactor. */
  private Reactor mainReactor;

  /** The command to run the generated code if specified in the target directive. */
  private List<String> runCommand;

  /** Parameters of the main reactor that can be overridden from the command line. */
  private List<Parameter> cliParams;

  /** Names of top-level parameters that are used for multiport or bank widths. */
  private Set<String> widthParams;

  public CMainFunctionGenerator(TargetConfig targetConfig, Reactor mainReactor) {
    this.targetConfig = targetConfig;
    this.mainReactor = mainReactor;
    runCommand = new ArrayList<>();
    cliParams = new ArrayList<>();
    widthParams = new HashSet<>();
    parseTargetParameters();
    collectCliParameters();
    collectWidthParameters();
  }

  /**
   * Generate the code that is the entry point of the program.
   *
   * <p>Ideally, this code would belong to its own `main.c` file, but it currently lives in
   * the same file as all the code generated for reactors.
   */
  public String generateCode() {
    CodeBuilder code = new CodeBuilder();
    code.pr(generateCliGlobals());
    code.pr(generateMainFunction());
    code.pr(generateSetDefaultCliOption());
    return code.toString();
  }

  /** Generate the `main` function. */
  private String generateMainFunction() {
    var platform = Platform.AUTO;
    if (targetConfig.isSet(PlatformProperty.INSTANCE)) {
      platform = targetConfig.get(PlatformProperty.INSTANCE).platform();
    }
    switch (platform) {
      case ARDUINO -> {
        /**
         * By default, we must have a serial begin line prior to calling lf_reactor_c_main due to
         * internal debugging messages requiring a print buffer. For the future, we can check
         * whether internal LF logging is enabled or not before removing this line. - Logging
         */
        return String.join(
            "\n",
            "\nvoid _lf_arduino_print_message_function(const char* format, va_list args) {",
            "\tchar buf[128];",
            "\tvsnprintf(buf, 128, format, args);",
            "\tSerial.print(buf);",
            "}\n",
            "// Arduino setup() and loop() functions",
            "void setup() {",
            "\tSerial.begin("
                + targetConfig.get(PlatformProperty.INSTANCE).baudRate().value()
                + ");",
            "\tlf_register_print_function(&_lf_arduino_print_message_function, LOG_LEVEL);",
            "\tlf_reactor_c_main(0, NULL);",
            "}\n",
            "void loop() {}");
      }
      case ZEPHYR -> {
        // The Zephyr "runtime" does not terminate when main returns.
        //  Rather, `exit` should be called explicitly.
        return String.join(
            "\n",
            "int main(void) {",
            "   int res = lf_reactor_c_main(0, NULL);",
            "   exit(res);",
            "   return 0;",
            "}");
      }
      case RP2040 -> {
        return String.join("\n", "int main(void) {", "   return lf_reactor_c_main(0, NULL);", "}");
      }
      default -> {
        if (cliParams.isEmpty()) {
          return String.join(
              "\n",
              "int main(int argc, const char* argv[]) {",
              "    return lf_reactor_c_main(argc, argv);",
              "}");
        }
        return generateMainWithCliParsing();
      }
    }
  }

  /**
   * Generate a main() function that pre-processes command-line arguments to extract
   * user-defined main reactor parameters before forwarding the remaining arguments
   * to lf_reactor_c_main().
   */
  private String generateMainWithCliParsing() {
    var code = new CodeBuilder();
    code.pr("int main(int argc, const char* argv[]) {");
    code.indent();
    code.pr("const char** newargv = (const char**)malloc(argc * sizeof(const char*));");
    code.pr("int newargc = 0;");
    code.pr("newargv[newargc++] = argv[0];");
    code.pr("for (int i = 1; i < argc; i++) {");
    code.indent();

    code.pr("if (strcmp(argv[i], \"--help\") == 0 || strcmp(argv[i], \"-h\") == 0) {");
    code.indent();
    code.pr(generateHelpMessage());
    code.pr("free(newargv);");
    code.pr("return 0;");
    code.unindent();

    for (Parameter param : cliParams) {
      var name = param.getName();
      var isTime = ASTUtils.isOfTimeType(param);

      code.pr("} else if (strcmp(argv[i], \"--" + name + "\") == 0) {");
      code.indent();

      if (widthParams.contains(name)) {
        code.pr(
            "fprintf(stderr, \"Error: Command-line changes to multiport and bank widths"
                + " are not supported.\\n"
                + "Change the width in the source code and recompile instead.\\n\");");
        code.pr("free(newargv);");
        code.pr("return 1;");
        code.unindent();
        continue;
      }

      if (isTime) {
        code.pr("if (i + 2 >= argc) {");
        code.indent();
        code.pr(
            "fprintf(stderr, \"Error: --"
                + name
                + " needs a time value and units (e.g., --"
                + name
                + " 500 msec).\\n\");");
        code.pr("free(newargv);");
        code.pr("return 1;");
        code.unindent();
        code.pr("}");
        code.pr("const char* time_str = argv[++i];");
        code.pr("const char* unit_str = argv[++i];");
        code.pr("if (lf_time_parse(time_str, unit_str, &_lf_cli_" + name + ") != 0) {");
        code.indent();
        code.pr(
            "fprintf(stderr, \"Error: invalid time value '%s %s' for --"
                + name
                + ".\\n\", time_str, unit_str);");
        code.pr("free(newargv);");
        code.pr("return 1;");
        code.unindent();
        code.pr("}");
        code.pr("_lf_cli_" + name + "_given = true;");
      } else {
        code.pr("if (i + 1 >= argc) {");
        code.indent();
        code.pr("fprintf(stderr, \"Error: --" + name + " needs a value.\\n\");");
        code.pr("free(newargv);");
        code.pr("return 1;");
        code.unindent();
        code.pr("}");
        code.pr("_lf_cli_" + name + " = atoi(argv[++i]);");
        code.pr("_lf_cli_" + name + "_given = true;");
      }

      code.unindent();
    }
    code.pr("} else {");
    code.indent();
    code.pr("newargv[newargc++] = argv[i];");
    code.unindent();
    code.pr("}");

    code.unindent();
    code.pr("}");
    code.pr("int ret = lf_reactor_c_main(newargc, newargv);");
    code.pr("free(newargv);");
    code.pr("return ret;");
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /**
   * Generate the printf statements for the --help message, listing all
   * user-defined main reactor parameters and the runtime options.
   */
  private String generateHelpMessage() {
    var code = new CodeBuilder();
    code.pr("printf(\"Usage: %s [options]\\n\\n\", argv[0]);");
    code.pr("printf(\"Reactor Parameters:\\n\");");
    for (Parameter param : cliParams) {
      var name = param.getName();
      var isTime = ASTUtils.isOfTimeType(param);
      if (isTime) {
        TimeValue defaultVal = ASTUtils.getDefaultAsTimeValue(param);
        String defaultStr = (defaultVal != null) ? defaultVal.toString() : "0";
        code.pr(
            "printf(\"  --"
                + name
                + " <value> <units>\\n"
                + "      time value (default: "
                + defaultStr
                + ")\\n\\n\");");
      } else {
        String defaultStr = "0";
        var expr = param.getInit().getExpr();
        if (expr instanceof Literal) {
          defaultStr = ((Literal) expr).getLiteral();
        }
        code.pr(
            "printf(\"  --"
                + name
                + " <value>\\n"
                + "      int value (default: "
                + defaultStr
                + ")\\n\\n\");");
      }
    }
    code.pr("printf(\"Runtime Options:\\n\");");
    code.pr(
        "printf(\"  -f, --fast <true|false>\\n"
            + "      Whether to wait for physical time to match logical time.\\n\\n\");");
    code.pr(
        "printf(\"  -o, --timeout <duration> <units>\\n"
            + "      Stop after the specified amount of logical time, where units are one of\\n"
            + "      nsec, usec, msec, sec, minute, hour, day, week, or the plurals of those.\\n"
            + "\\n"
            + "\");");
    code.pr(
        "printf(\"  -k, --keepalive <true|false>\\n"
            + "      Whether to continue execution even when there are no events to process.\\n"
            + "\\n"
            + "\");");
    code.pr(
        "printf(\"  -w, --workers <n>\\n"
            + "      Execute in <n> threads if possible (optional feature).\\n\\n\");");
    code.pr("printf(\"  -h, --help\\n      Display this help message.\\n\\n\");");
    return code.toString();
  }

  /**
   * Generate global variable declarations for CLI parameter overrides,
   * along with the necessary #include directives.
   */
  private String generateCliGlobals() {
    if (cliParams.isEmpty()) {
      return "";
    }
    var code = new CodeBuilder();
    code.pr("#include <string.h>");
    code.pr("#include <stdlib.h>");
    code.pr("#include <stdio.h>");
    for (Parameter param : cliParams) {
      var name = param.getName();
      var isTime = ASTUtils.isOfTimeType(param);
      if (isTime) {
        code.pr("interval_t _lf_cli_" + name + ";");
      } else {
        code.pr("int _lf_cli_" + name + ";");
      }
      code.pr("bool _lf_cli_" + name + "_given = false;");
    }
    return code.toString();
  }

  /**
   * Collect main reactor parameters that can be overridden from the command line.
   * Currently supports parameters of type 'time' and 'int'.
   */
  private void collectCliParameters() {
    if (mainReactor == null) {
      return;
    }
    // Embedded platforms have no command-line interface.
    if (targetConfig.isSet(PlatformProperty.INSTANCE)) {
      var platform = targetConfig.get(PlatformProperty.INSTANCE).platform();
      if (platform == Platform.ARDUINO
          || platform == Platform.ZEPHYR
          || platform == Platform.RP2040
          || platform == Platform.FLEXPRET) {
        return;
      }
    }
    for (Parameter param : ASTUtils.allParameters(mainReactor)) {
      if (ASTUtils.isOfTimeType(param)) {
        cliParams.add(param);
      } else if (param.getType() != null
          && !param.getType().isTime()
          && param.getType().getCStyleArraySpec() == null) {
        var baseType = ASTUtils.baseType(param.getType());
        if ("int".equals(baseType)) {
          cliParams.add(param);
        }
      }
    }
  }

  /**
   * Generate code that is used to override the command line options to the `main` function
   */
  private String generateSetDefaultCliOption() {
    // Generate function to set default command-line options.
    // A literal array needs to be given outside any function definition,
    // so start with that.
    return runCommand.size() > 0
        ? String.join(
            "\n",
            "const char* _lf_default_argv[] = { "
                + StringUtil.joinObjects(
                    runCommand.stream().map(StringUtil::addDoubleQuotes).toList(), ", ")
                + " };",
            "void lf_set_default_command_line_options() {",
            "        default_argc = " + runCommand.size() + ";",
            "        default_argv = _lf_default_argv;",
            "}")
        : "void lf_set_default_command_line_options() {}";
  }

  /** Return the list of main reactor parameters that can be overridden from the command line. */
  public List<Parameter> getCliParameters() {
    return cliParams;
  }

  /** Parse the target parameters and set flags to the runCommand accordingly. */
  private void parseTargetParameters() {
    if (targetConfig.get(FastProperty.INSTANCE)) {
      runCommand.add("-f");
      runCommand.add("true");
    }
    if (targetConfig.get(KeepaliveProperty.INSTANCE)) {
      runCommand.add("-k");
      runCommand.add("true");
    }
    if (targetConfig.isSet(TimeOutProperty.INSTANCE)) {
      runCommand.add("-o");
      runCommand.add(targetConfig.get(TimeOutProperty.INSTANCE).getMagnitude() + "");
      runCommand.add(targetConfig.get(TimeOutProperty.INSTANCE).unit.getCanonicalName());
    }
    // The runCommand has a first entry that is ignored but needed.
    if (runCommand.size() > 0) {
      runCommand.add(0, "dummy");
    }
  }

  /**
   * Collect names of top-level parameters that are transitively used as multiport
   * widths or bank widths. Overriding these from the command line is not supported
   * because the connection topology is determined at compile time.
   */
  private void collectWidthParameters() {
    if (mainReactor == null) return;
    Set<String> mainParamNames = new HashSet<>();
    for (Parameter p : cliParams) mainParamNames.add(p.getName());
    if (mainParamNames.isEmpty()) return;

    Map<String, String> identity = new HashMap<>();
    for (String n : mainParamNames) identity.put(n, n);
    findWidthParams(mainReactor, mainParamNames, identity, widthParams, new HashSet<>());
  }

  /**
   * Recursively search for parameters used in port or bank width specs, tracing
   * parameter assignments through instantiations back to the main reactor.
   *
   * @param reactor       The reactor to inspect.
   * @param trackedParams Names of parameters in this reactor that originate from the main reactor.
   * @param toMainParam   Maps a tracked parameter name in this reactor to the originating
   *                      main-reactor parameter name.
   * @param result        Accumulates main-reactor parameter names that influence widths.
   * @param visited       Reactor definitions already visited (to avoid infinite recursion).
   */
  private void findWidthParams(
      Reactor reactor,
      Set<String> trackedParams,
      Map<String, String> toMainParam,
      Set<String> result,
      Set<Reactor> visited) {
    if (!visited.add(reactor)) return;

    for (Port port : ASTUtils.allPorts(reactor)) {
      WidthSpec ws = port.getWidthSpec();
      if (ws == null) continue;
      for (WidthTerm term : ws.getTerms()) {
        Parameter p = term.getParameter();
        if (p != null && trackedParams.contains(p.getName())) {
          result.add(toMainParam.get(p.getName()));
        }
      }
    }

    for (Instantiation inst : ASTUtils.allInstantiations(reactor)) {
      WidthSpec bws = inst.getWidthSpec();
      if (bws != null) {
        for (WidthTerm term : bws.getTerms()) {
          Parameter p = term.getParameter();
          if (p != null && trackedParams.contains(p.getName())) {
            result.add(toMainParam.get(p.getName()));
          }
        }
      }

      Reactor childReactor = ASTUtils.toDefinition(inst.getReactorClass());
      if (childReactor == null) continue;

      Set<String> childTracked = new HashSet<>();
      Map<String, String> childToMain = new HashMap<>();
      for (Assignment assign : inst.getParameters()) {
        var rhs = assign.getRhs().getExpr();
        if (rhs instanceof ParameterReference pr) {
          String srcName = pr.getParameter().getName();
          if (trackedParams.contains(srcName)) {
            String childName = assign.getLhs().getName();
            childTracked.add(childName);
            childToMain.put(childName, toMainParam.get(srcName));
          }
        }
      }
      if (!childTracked.isEmpty()) {
        findWidthParams(childReactor, childTracked, childToMain, result, visited);
      }
    }
  }
}
