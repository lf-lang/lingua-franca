package org.lflang.generator.c;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.generator.TargetTypes;

public class CTypes implements TargetTypes {

    // Regular expression pattern for array types.
    // For example, for "foo[10]", the first match will be "foo" and the second "[10]".
    // For "foo[]", the first match will be "foo" and the second "".
    static final Pattern arrayPattern = Pattern.compile("^\\s*(\\w+)\\s*\\[([0-9]*)]\\s*$");

    // FIXME: Instead of using the ErrorReporter, perhaps we should be raising assertion errors or
    //  UnsupportedOperationExceptions or some other non-recoverable errors.
    private final ErrorReporter errorReporter;

    /**
     * Initializes a {@code CTargetTypes} with the given
     * error reporter.
     * @param errorReporter The error reporter for any
     *                      errors raised in the code
     *                      generation process.
     */
    public CTypes(ErrorReporter errorReporter) {
        this.errorReporter = errorReporter;
    }

    @Override
    public boolean supportsGenerics() {
        return false;
    }

    @Override
    public String getTargetTimeType() {
        return "interval_t";
    }

    @Override
    public String getTargetTagType() {
        return "tag_t";
    }

    @Override
    public String getTargetFixedSizeListType(String baseType, int size) {
        return String.format("%s[%d]", baseType, size);
    }

    @Override
    public String getTargetVariableSizeListType(String baseType) {
        return String.format("%s[]", baseType);
    }

    @Override
    public String getTargetUndefinedType() {
        return String.format("/* %s */", errorReporter.reportError("undefined type"));
    }

    @Override
    public String getTargetType(InferredType type) {
        var result = TargetTypes.super.getTargetType(type);
        // FIXME: This is brittle. Rather than patching up the results of almost-right calls
        //  to other methods, we should have a more general implementation of the method that
        //  takes more parameters.
        Matcher matcher = arrayPattern.matcher(result);
        if (matcher.find()) {
            return matcher.group(1) + '*';
        }
        return result;
    }

    /**
     * Provides the same functionality as getTargetType, except with
     * the array syntax {@code []} preferred over the pointer syntax
     * {@code *} (if applicable), and with the variable name
     * included.
     * The result is of the form {@code type variable_name[]} or
     * {@code type variable_name} depending on whether the given type
     * is an array type.
     */
    public String getVariableDeclaration(InferredType type, String variableName) {
        String t = getTargetType(type);
        Matcher matcher = arrayPattern.matcher(t);
        String declaration = String.format("%s %s", t, variableName);
        if (matcher.find()) {
            // If the state type ends in [], then we have to move the []
            // because C is very picky about where this goes. It has to go
            // after the variable name.
            declaration = String.format("%s %s[%s]", matcher.group(1), variableName, matcher.group(2));
        }
        return declaration;
    }
}
