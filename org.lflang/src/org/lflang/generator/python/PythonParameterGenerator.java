package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import com.google.common.base.Objects;

import org.lflang.ASTUtils;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ParameterInstance;
import org.lflang.lf.Expression;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Assignment;
import org.lflang.lf.Parameter;


public class PythonParameterGenerator {
    /**
     * Generate Python code that instantiates and initializes parameters for a reactor 'decl'.
     *
     * @param decl The reactor declaration
     * @return The generated code as a StringBuilder
     */
    public static String generatePythonInstantiations(ReactorDecl decl, PythonTypes types) {
        List<String> lines = new ArrayList<>();
        lines.add("# Define parameters and their default values");

        for (Parameter param : getAllParameters(decl)) {
            if (!types.getTargetType(param).equals("PyObject*")) {
                // If type is given, use it
                String type = types.getPythonType(ASTUtils.getInferredType(param));
                lines.add("self._"+param.getName()+":"+type+" = "+generatePythonInitializer(param));
            } else {
                // If type is not given, just pass along the initialization
                lines.add("self._"+param.getName()+" = "+generatePythonInitializer(param));
            }
        }
        // Handle parameters that are set in instantiation
        lines.addAll(List.of(
            "# Handle parameters that are set in instantiation",
            "self.__dict__.update(kwargs)",
            ""
        ));
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
                lines.addAll(List.of(
                    "",
                    "@property",
                    "def "+param.getName()+"(self):",
                    "    return self._"+param.getName()+" # pylint: disable=no-member",
                    ""
                ));
            }
        }
        // Create a special property for bank_index
        lines.addAll(List.of(
            "",
            "@property",
            "def bank_index(self):",
            "    return self._bank_index # pylint: disable=no-member",
            ""
        ));
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
        List<String> values = p.getInit().getExprs().stream().map(PyUtil::getPythonTargetValue).toList();
        return values.size() > 1 ? "(" + String.join(", ", values) + ")" : values.get(0);
    }

    /**
     * Return a Python expression that can be used to initialize the specified
     * parameter instance. If the parameter initializer refers to other
     * parameters, then those parameter references are replaced with
     * accesses to the Python reactor instance class of the parents of
     * those parameters.
     *
     * @param p The parameter instance to create initializer for
     * @return Initialization code
     */
    public static String generatePythonInitializer(ParameterInstance p) {
        // Handle overrides in the instantiation.
        // In case there is more than one assignment to this parameter, we need to
        // find the last one.
        Assignment lastAssignment = getLastAssignment(p);
        List<String> list = new LinkedList<>();
        if (lastAssignment != null) {
            // The parameter has an assignment.
            // Right hand side can be a list. Collect the entries.
            for (Expression expr : lastAssignment.getRhs().getExprs()) {
                if (expr instanceof ParameterReference) {
                    // The parameter is being assigned a parameter value.
                    // Assume that parameter belongs to the parent's parent.
                    // This should have been checked by the validator.
                    final var param = ((ParameterReference) expr).getParameter();
                    list.add(PyUtil.reactorRef(p.getParent().getParent()) + "." + param.getName());
                } else {
                    list.add(GeneratorBase.getTargetTime(expr));
                }
            }
        } else {
            for (Expression expr : p.getParent().initialParameterValue(p.getDefinition())) {
                list.add(PyUtil.getPythonTargetValue(expr));
            }
        }
        return list.size() > 1 ? "(" + String.join(", ", list) + ")" : list.get(0);
    }

    /**
     * Returns the last assignment to "p" if there is one,
     * or null if there is no assignment to "p"
     *
     * @param p The parameter instance to create initializer for
     * @return The last assignment of the parameter instance
     */
    private static Assignment getLastAssignment(ParameterInstance p) {
        Assignment lastAssignment = null;
        for (Assignment assignment : p.getParent().getDefinition().getParameters()) {
            if (Objects.equal(assignment.getLhs(), p.getDefinition())) {
                lastAssignment = assignment;
            }
        }
        return lastAssignment;
    }
}
