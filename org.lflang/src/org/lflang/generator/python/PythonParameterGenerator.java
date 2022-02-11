package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.JavaAstUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.CParameterGenerator;
import org.lflang.generator.python.PythonTypes;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Parameter;


public class PythonParameterGenerator {
    /**
     * Generate runtime initialization code in C for parameters of a given reactor instance.
     * All parameters are also initialized in Python code, but those parameters that are
     * used as width must be also initialized in C.
     * 
     * FIXME: Here, we use a hack: we attempt to convert the parameter initialization to an integer.
     * If it succeeds, we proceed with the C initialization. If it fails, we defer initialization
     * to Python.
     * 
     * Generate runtime initialization code for parameters of a given reactor instance
     * @param instance The reactor instance.
     */
    public static void _generateParameterInitialization(ReactorInstance instance,
                                                        CodeBuilder initializeTriggerObjects) {
        // Mostly ignore the initialization in C
        // The actual initialization will be done in Python 
        // Except if the parameter is a width (an integer)
        // Here, we attempt to convert the parameter value to 
        // integer. If it succeeds, we also initialize it in C.
        // If it fails, we defer the initialization to Python.
        String nameOfSelfStruct = CUtil.reactorRef(instance);
        for (ParameterInstance parameter : instance.parameters) {
            String initializer = CParameterGenerator.getInitializer(parameter);
            try {
                // Attempt to convert it to integer
                int number = Integer.parseInt(initializer);
                initializeTriggerObjects.pr(String.join("\n", 
                "",
                "    "+nameOfSelfStruct+"->"+parameter.getName()+" = "+number+";",
                ""));
            } catch (NumberFormatException ex) {
                // Ignore initialization in C for this parameter
            }
        }
    }

    /**
     * Generate Python code that instantiates and initializes parameters for a reactor 'decl'.
     * 
     * @param decl The reactor declaration
     * @return The generated code as a StringBuilder
     */
    public static String generateParametersForPython(ReactorDecl decl, PythonTypes types) {
        List<String> lines = new ArrayList<>();
        lines.add("# Define parameters and their default values");
        
        for (Parameter param : ASTUtils.allParameters(ASTUtils.toDefinition(decl))) {
            if (!types.getTargetType(param).equals("PyObject*")) {
                // If type is given, use it
                lines.add("self._"+param.getName()+":"+types.getPythonType(JavaAstUtils.getInferredType(param))+" = "+getPythonInitializer(param));
            } else {
                // If type is not given, just pass along the initialization
                lines.add("self._"+param.getName()+" = "+getPythonInitializer(param));
            }
        }

        // Handle parameters that are set in instantiation
        lines.add("# Handle parameters that are set in instantiation");
        lines.add("self.__dict__.update(kwargs)\n");
        return String.join("\n", lines);
    }

    /**
     * Create a Python list for parameter initialization in target code.
     * 
     * @param p The parameter to create initializers for
     * @return Initialization code
     */
    private static String getPythonInitializer(Parameter p) {
        if (p.getInit().size() > 1) {
            // parameters are initialized as immutable tuples
            List<String> targetValues = p.getInit().stream().map(it -> PyUtil.getPythonTargetValue(it)).collect(Collectors.toList());
            return "(" + String.join(", ", targetValues) + ")";
        } else {
            return PyUtil.getPythonTargetValue(p.getInit().get(0));
        }
    }
}
