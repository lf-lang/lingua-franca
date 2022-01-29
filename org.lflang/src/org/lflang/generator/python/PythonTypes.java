package org.lflang.generator.python;

import java.util.regex.Pattern;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.Initializer;
import org.lflang.lf.Literal;
import org.lflang.lf.ParamRef;
import org.lflang.lf.Type;

public class PythonTypes extends CTypes {

    // Regular expression pattern for pointer types. The star at the end has to be visible.
    static final Pattern pointerPatternVariable = Pattern.compile("^\\s*+(\\w+)\\s*\\*\\s*$");
    public static final PythonTypes INSTANCE = new PythonTypes();

    protected PythonTypes() {
    }

    @Override
    public String getTargetUndefinedType() {
        return "PyObject*";
    }

    @Override
    public String getTargetLiteral(Literal expr, InferredType type) {
        switch (expr.getLiteral()) {
        case "False":
        case "True":
            return expr.getLiteral();
        default:
            return super.getTargetLiteral(expr, type);
        }
    }

    @Override
    public String getTargetParamRef(ParamRef expr, InferredType type) {
        return "self." + expr.getParameter().getName();
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
