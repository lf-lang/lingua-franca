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
    static final Pattern arrayPattern = Pattern.compile("^\\s*(?:/\\*.*?\\*/)?\\s*(\\w+)\\s*\\[([0-9]*)]\\s*$");

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

    /**
     * Given a type, return a C representation of the type. Note that
     * C types are very idiosyncratic. For example, {@code int[]} is not always accepted
     * as a type, and {@code int*} must be used instead, except when initializing
     * a variable using a static initializer, as in {@code int[] foo = {1, 2, 3};}.
     * When initializing a variable using a static initializer, use
     * {@link #getVariableDeclaration(InferredType, String)} instead.
     * @param type The type.
     */
    @Override
    public String getTargetType(InferredType type) {
        var result = TargetTypes.super.getTargetType(type);
        Matcher matcher = arrayPattern.matcher(result);
        if (matcher.find()) {
            return matcher.group(1) + '*';
        }
        return result;
    }

    /**
     * Return a variable declaration of the form "{@code type name}".
     * The type is as returned by {@link #getTargetType(InferredType)}, except with
     * the array syntax {@code [size]} preferred over the pointer syntax
     * {@code *} (if applicable). This also includes the variable name
     * because C requires the array type specification to be placed after
     * the variable name rather than with the type.
     * The result is of the form {@code type variable_name[size]} or
     * {@code type variable_name} depending on whether the given type
     * is an array type, unless the array type has no size (it is given
     * as {@code []}. In that case, the returned form depends on the
     * third argument, initializer. If true, the then the returned
     * declaration will have the form {@code type variable_name[]},
     * and otherwise it will have the form {@code type* variable_name}.
     * @param type The type.
     * @param variableName The name of the variable.
     * @param initializer True to return a form usable in a static initializer.
     */
    public String getVariableDeclaration(
            InferredType type,
            String variableName,
            boolean initializer
    ) {
        String t = TargetTypes.super.getTargetType(type);
        Matcher matcher = arrayPattern.matcher(t);
        String declaration = String.format("%s %s", t, variableName);
        if (matcher.find()) {
            // For array types, we have to move the []
            // because C is very picky about where this goes. It has to go
            // after the variable name. Also, in an initializer, it has to have
            // form [], and in a struct definition, it has to use *.
            if (matcher.group(2).equals("") && !initializer) {
                declaration = String.format("%s* %s",
                        matcher.group(1), variableName);
            } else {
                declaration = String.format("%s %s[%s]",
                        matcher.group(1), variableName, matcher.group(2));
            }
        }
        return declaration;
    }
}
