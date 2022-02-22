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

package org.lflang;

import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import org.lflang.lf.Action;
import org.lflang.lf.BraceExpr;
import org.lflang.lf.BracketExpr;
import org.lflang.lf.Initializer;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Literal;
import org.lflang.lf.ParamRef;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.lf.Type;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;

/**
 * Helper class to manipulate the LF AST. This is partly
 * converted from {@link ASTUtils}.
 */
public final class JavaAstUtils {
    /* Match an abbreviated form of a float literal. */
    private static final Pattern ABBREVIATED_FLOAT = Pattern.compile("[+\\-]?\\.\\d+[\\deE+\\-]*");

    private JavaAstUtils() {
        // utility class
    }

    /**
     * Returns the time value represented by the given AST node.
     */
    public static TimeValue toTimeValue(Time e) {
        if (!isValidTime(e)) {
            // invalid unit, will have been reported by validator
            throw new IllegalArgumentException();
        }
        return new TimeValue(e.getInterval(), TimeUnit.fromName(e.getUnit()));
    }

    /**
     * Format the initializer as it would appear in LF.
     */
    public static String toText(Initializer init) {
        if (init.isBraces()) {
            return init.getExprs().stream()
                       .map(ASTUtils::toText)
                       .collect(Collectors.joining(", ", "{", "}"));
        } else if (init.isParens()) {
            return init.getExprs().stream()
                       .map(ASTUtils::toText)
                       .collect(Collectors.joining(", ", "(", ")"));
        } else if (init.isAssign()) {
            return "= " + ASTUtils.toText(asSingleValue(init));
        } else {
            return ""; // no initializer
        }
    }


    /**
     * If the given string can be recognized as a floating-point number that has a leading decimal point,
     * prepend the string with a zero and return it. Otherwise, return the original string.
     *
     * @param literal A string might be recognizable as a floating point number with a leading decimal point.
     * @return an equivalent representation of <code>literal
     * </code>
     */
    public static String addZeroToLeadingDot(String literal) {
        Matcher m = ABBREVIATED_FLOAT.matcher(literal);
        if (m.matches()) {
            return literal.replace(".", "0.");
        }
        return literal;
    }


    ////////////////////////////////
    //// Utility functions for translating AST nodes into text
    // This is a continuation of a large section of ASTUtils.xtend
    // with the same name.

    /**
     * Generate code for referencing a port, action, or timer.
     * @param reference The reference to the variable.
     */
    public static String generateVarRef(VarRef reference) {
        var prefix = "";
        if (reference.getContainer() != null) {
            prefix = reference.getContainer().getName() + ".";
        }
        return prefix + reference.getVariable().getName();
    }

    /**
     * Assuming that the given value denotes a valid time literal,
     * return a time value.
     */
    public static TimeValue getLiteralTimeValue(Value v) {
        if (v instanceof Time) {
            return toTimeValue((Time) v);
        } else if (v instanceof Literal && ASTUtils.isZero(v)) {
            return TimeValue.ZERO;
        } else {
            return null;
        }
    }

    public static TimeValue getTimeValue(Value v) {
        if (v instanceof Time) {
            return toTimeValue((Time) v);
        } else if (v instanceof Literal && ASTUtils.isZero(v)) {
            return TimeValue.ZERO;
        } else {
            return null;
        }
    }

    /**
     * If the parameter is of time type, return its default value.
     * Otherwise, return null.
     */
    public static TimeValue getDefaultAsTimeValue(Parameter p) {
        if (isOfTimeType(p)) {
            var init = asSingleValue(p.getInit());
            if (init != null) {
                return getLiteralTimeValue(init);
            }
        }
        return null;
    }

    /**
     * Return whether the given state variable is inferred
     * to a time type.
     */
    public static boolean isOfTimeType(StateVar state) {
        InferredType t = getInferredType(state);
        return t.isTime && !t.isList;
    }

    /**
     * Return whether the given parameter is inferred
     * to a time type.
     */
    public static boolean isOfTimeType(Parameter param) {
        InferredType t = getInferredType(param);
        return t.isTime && !t.isList;
    }

    /**
     * Returns true if the argument denotes a valid time, false otherwise.
     *
     * @param t AST node to inspect (non-null).
     */
    public static boolean isValidTime(Time t) {
        return t != null && TimeUnit.isValidUnit(t.getUnit())
            && (t.getUnit() != null || t.getInterval() == 0);
    }

    /**
     * Given an initializer that is known to be of a list type
     * (because of a type annotation), return the components of
     * the list. Return null if the initializer is not a list.
     */
    public static List<Value> initializerAsList(Initializer init) {
        if (init.isAssign()) {
            var list = asSingleValue(init);
            if (list instanceof BracketExpr) {
                return ((BracketExpr) list).getItems();
            } else if (list instanceof BraceExpr) {
                return ((BraceExpr) list).getItems();
            } else {
                return null;
            }
        } else {
            return init.getExprs();
        }
    }

    public static Initializer listAsInitializer(List<Value> values) {
        // for compatibility with ParameterInstance
        Initializer initializer = LfFactory.eINSTANCE.createInitializer();
        initializer.setParens(true);
        initializer.getExprs().addAll(values);
        return initializer;
    }


}
