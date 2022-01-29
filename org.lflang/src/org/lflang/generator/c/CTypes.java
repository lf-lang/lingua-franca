/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.c;

import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import org.lflang.InferredType;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.UnsupportedGeneratorFeatureException;
import org.lflang.lf.Initializer;

/**
 * {@link TargetTypes} impl for {@link CGenerator}.
 *
 * @author Clément Fournier
 */
public class CTypes implements TargetTypes {

    // Regular expression pattern for array types.
    // For example, for "foo[10]", the first match will be "foo" and the second "[10]".
    // For "foo[]", the first match will be "foo" and the second "".
    private static final Pattern arrayPattern = Pattern.compile("^\\s*(?:/\\*.*?\\*/)?\\s*(\\w+)\\s*\\[([0-9]*)]\\s*$");


    public static final CTypes INSTANCE = new CTypes();

    protected CTypes() {
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
        return baseType + "[" + size + "]";
    }

    @Override
    public String getTargetVariableSizeListType(String baseType) {
        return baseType + "[]";
    }

    @Override
    public String getTargetUndefinedType() {
        // todo C used to insert a marker in the code
        // return String.format("/* %s */", errorReporter.reportError("undefined type"));
        throw new UnsupportedGeneratorFeatureException("Undefined type");
    }

    @Override
    public String getTargetTimeExpr(TimeValue timeValue) {
        if (timeValue.equals(TimeValue.ZERO)) {
            return "0";
        }
        return cMacroName(timeValue.getUnit()) + '(' + timeValue.getMagnitude() + ')';
    }

    private static String cMacroName(TimeUnit unit) {
        return unit.getCanonicalName().toUpperCase(Locale.ROOT);
    }

    @Override
    public String getMissingExpr(InferredType type) {
        return "0";
    }

    @Override
    public String getTargetInitializerWithNotExactlyOneValue(Initializer init, InferredType type) {
        return init.getExprs().stream()
                   .map(it -> getTargetExpr(it, type.getComponentType()))
                   .collect(Collectors.joining(", ", "{", "}"));
    }

    /**
     * Given a type, return a C representation of the type. Note that
     * C types are very idiosyncratic. For example, {@code int[]} is not always accepted
     * as a type, and {@code int*} must be used instead, except when initializing
     * a variable using a static initializer, as in {@code int[] foo = {1, 2, 3};}.
     * When initializing a variable using a static initializer, use
     * {@link #getVariableDeclaration(InferredType, String, boolean)} instead.
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
