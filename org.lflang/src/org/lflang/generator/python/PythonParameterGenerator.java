package org.lflang.generator.python;

import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.CParameterGenerator;


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
}
