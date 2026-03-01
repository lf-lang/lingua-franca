package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.ParameterInstance;
import org.lflang.lf.Parameter;
import org.lflang.lf.ReactorDecl;

/**
 * Generate Python code for parameters.
 *
 * @ingroup Generator
 */
public class PythonParameterGenerator {
  /**
   * Generate Python code that instantiates and initializes parameters for a reactor 'decl'.
   *
   * @param decl The reactor declaration
   * @param types The Python type helper.
   * @return The generated code as a StringBuilder
   */
  public static String generatePythonInstantiations(ReactorDecl decl, PythonTypes types) {
    List<String> lines = new ArrayList<>();
    lines.add("# Define parameters and their default values");

    for (Parameter param : getAllParameters(decl)) {
      if (!types.getTargetType(param).equals("PyObject*")) {
        // If type is given, use it
        String type = types.getPythonType(ASTUtils.getInferredType(param));
        lines.add(
            "self._" + param.getName() + ":" + type + " = " + generatePythonInitializer(param));
      } else {
        // If type is not given, just pass along the initialization
        lines.add("self._" + param.getName() + " = " + generatePythonInitializer(param));
      }
    }
    // Handle parameters that are set in instantiation
    lines.addAll(
        List.of(
            "# Handle parameters that are set in instantiation",
            "self.__dict__.update(kwargs)",
            ""));
    return String.join("\n", lines);
  }

  /**
   * Generate Python code getters for parameters of reactor 'decl'.
   *
   * @param decl The reactor declaration
   * @return The generated code as a StringBuilder
   */
  public static String generatePythonGetters(ReactorDecl decl) {
    List<String> lines = new ArrayList<>();
    for (Parameter param : getAllParameters(decl)) {
      if (!param.getName().equals("bank_index")) {
        lines.addAll(
            List.of(
                "",
                "@property",
                "def " + param.getName() + "(self):",
                "    return self._" + param.getName() + " # pylint: disable=no-member",
                ""));
      }
    }
    // Create a special property for bank_index
    lines.addAll(
        List.of(
            "",
            "@property",
            "def bank_index(self):",
            "    return self._bank_index # pylint: disable=no-member",
            ""));
    lines.add("\n");
    return String.join("\n", lines);
  }

  /**
   * Return a list of all parameters of reactor 'decl'.
   *
   * @param decl The reactor declaration
   * @return The list of all parameters of 'decl'
   */
  private static List<Parameter> getAllParameters(ReactorDecl decl) {
    return ASTUtils.allParameters(ASTUtils.toDefinition(decl));
  }

  /**
   * Create a Python list for parameter initialization in target code.
   *
   * @param p The parameter to create initializers for
   * @return Initialization code
   */
  private static String generatePythonInitializer(Parameter p) {
    return PythonTypes.getInstance().getTargetInitializer(p.getInit(), p.getType());
  }

  /**
   * Return a Python expression that can be used to initialize the specified parameter instance. If
   * the parameter initializer refers to other parameters, then those parameter references are
   * replaced with accesses to the Python reactor instance class of the parents of those parameters.
   *
   * @param p The parameter instance to create initializer for
   * @return Initialization code
   */
  public static String generatePythonInitializer(ParameterInstance p) {
    PythonTypes pyTypes = PythonTypes.generateParametersIn(p.getParent().getParent());
    return pyTypes.getTargetInitializer(p.getActualValue(), p.getDefinition().getType());
  }
}
