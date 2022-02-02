package org.lflang.generator.python;

import java.util.regex.Pattern;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.generator.c.CTypes;

public class PythonTypes extends CTypes {

    // Regular expression pattern for pointer types. The star at the end has to be visible.
    static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");

    /**
     * Initializes a {@code CTargetTypes} with the given
     * error reporter.
     *
     * @param errorReporter The error reporter for any
     *                      errors raised in the code
     *                      generation process.
     */
    public PythonTypes(ErrorReporter errorReporter) {
        super(errorReporter);
    }

    @Override
    public String getTargetUndefinedType() {
        return "PyObject*";
    }

    /**
     * This generator inherits types from the CGenerator.
     * This function reverts them back to Python types
     * For example, the types double is converted to float,
     * the * for pointer types is removed, etc.
     * @param type The type
     * @return The Python equivalent of a C type
     */
    public String getPythonType(InferredType type) {
        var result = super.getTargetType(type);

        switch(result){
        case "double": result = "float";
        case "string": result = "object";
        }

        var matcher = pointerPatternVariable.matcher(result);
        if(matcher.find()) {
            return matcher.group(1);
        }

        return result;
    }
}
