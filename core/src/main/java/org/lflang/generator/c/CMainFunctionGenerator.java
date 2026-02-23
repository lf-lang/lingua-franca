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
   * Generate a main() function that uses the table-driven process_user_args()
   * to extract user-defined main reactor parameters before forwarding the
   * remaining arguments to lf_reactor_c_main().
   */
  private String generateMainWithCliParsing() {
    var code = new CodeBuilder();
    code.pr("int main(int argc, const char* argv[]) {");
    code.indent();
    code.pr("_lf_cli_params = _lf_cli_params_table;");
    code.pr("_lf_cli_params_count = " + cliParams.size() + ";");
    code.pr("// Use a fixed-size array because MSVC does not support variable-length arrays.");
    code.pr("#define MAX_ARGV 64");
    code.pr("if (argc > MAX_ARGV) {");
    code.indent();
    code.pr("lf_print_error(\"Too many command-line arguments (max %d).\", MAX_ARGV);");
    code.pr("return 1;");
    code.unindent();
    code.pr("}");
    code.pr("const char* newargv[MAX_ARGV];");
    code.pr("int newargc = 0;");
    code.pr("int result = process_user_args(argc, argv, &newargc, newargv);");
    code.pr("if (result != 0) return (result == 1) ? 0 : 1;");
    code.pr("return lf_reactor_c_main(newargc, newargv);");
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /**
   * Generate global variable declarations for CLI parameter overrides
   * and the parameter descriptor table used by process_user_args().
   */
  private String generateCliGlobals() {
    if (cliParams.isEmpty()) {
      return "";
    }
    var code = new CodeBuilder();

    // Declare storage variables for each parameter.
    for (Parameter param : cliParams) {
      var name = param.getName();
      code.pr(cTypeFor(param) + " _lf_cli_" + name + ";");
      code.pr("bool _lf_cli_" + name + "_given = false;");
    }

    // Generate the parameter descriptor table.
    code.pr("lf_cli_param_t _lf_cli_params_table[] = {");
    code.indent();
    for (Parameter param : cliParams) {
      var name = param.getName();
      var isWidth = widthParams.contains(name);
      String description = descriptionFor(param);
      code.pr(
          "{\""
              + name
              + "\", "
              + cliTypeEnumFor(param)
              + ", &_lf_cli_"
              + name
              + ", &_lf_cli_"
              + name
              + "_given, \""
              + description
              + "\", "
              + isWidth
              + "},");
    }
    code.unindent();
    code.pr("};");
    return code.toString();
  }

  /** Supported scalar types for command-line overrides. */
  private static final Set<String> SUPPORTED_CLI_TYPES =
      Set.of("int", "double", "float", "bool", "string");

  /**
   * Collect main reactor parameters that can be overridden from the command line.
   * Supports time and stringparameters and scalar parameters of type int, double, float, and bool.
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
        if (SUPPORTED_CLI_TYPES.contains(baseType)) {
          cliParams.add(param);
        }
      }
    }
  }

  /** Return the C type string for a CLI parameter. */
  private String cTypeFor(Parameter param) {
    if (ASTUtils.isOfTimeType(param)) return "interval_t";
    var baseType = ASTUtils.baseType(param.getType());
    return switch (baseType) {
      case "double" -> "double";
      case "float" -> "float";
      case "bool" -> "bool";
      case "string" -> "const char*";
      default -> "int";
    };
  }

  /** Return the lf_cli_type_t enum constant name for a CLI parameter. */
  private String cliTypeEnumFor(Parameter param) {
    if (ASTUtils.isOfTimeType(param)) return "CLI_TIME";
    var baseType = ASTUtils.baseType(param.getType());
    return switch (baseType) {
      case "double" -> "CLI_DOUBLE";
      case "float" -> "CLI_FLOAT";
      case "bool" -> "CLI_BOOL";
      case "string" -> "CLI_STRING";
      default -> "CLI_INT";
    };
  }

  /** Return a human-readable description string for a CLI parameter's help message. */
  private String descriptionFor(Parameter param) {
    if (ASTUtils.isOfTimeType(param)) {
      TimeValue defaultVal = ASTUtils.getDefaultAsTimeValue(param);
      String defaultStr = (defaultVal != null) ? defaultVal.toString() : "0";
      return "time value (default: " + defaultStr + ")";
    }
    var baseType = ASTUtils.baseType(param.getType());
    String defaultStr = "unspecified";
    var init = param.getInit();
    if (init != null) {
      var expr = init.getExpr();
      if (expr instanceof Literal) {
        defaultStr = ((Literal) expr).getLiteral();
      }
      // Escape double quotes in the default string (e.g., string literals).
      defaultStr = defaultStr.replace("\"", "\\\"");
    }
    return baseType + " value (default: " + defaultStr + ")";
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
   * @param stack         Recursion stack for cycle detection. Unlike a global visited set, entries
   *                      are removed after returning so that the same reactor definition can be
   *                      analyzed multiple times with different parameter mappings.
   */
  private void findWidthParams(
      Reactor reactor,
      Set<String> trackedParams,
      Map<String, String> toMainParam,
      Set<String> result,
      Set<Reactor> stack) {
    if (!stack.add(reactor)) return;

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
        findWidthParams(childReactor, childTracked, childToMain, result, stack);
      }
    }
    stack.remove(reactor);
  }
}
