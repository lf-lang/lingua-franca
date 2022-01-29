/*
 * Copyright (c) 2021, The Authors of this file and their respective Institutions.
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


package org.lflang.generator;

import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.JavaAstUtils;
import org.lflang.Target;
import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.AddExpr;
import org.lflang.lf.BraceExpr;
import org.lflang.lf.BracketExpr;
import org.lflang.lf.Code;
import org.lflang.lf.CodeExpr;
import org.lflang.lf.Initializer;
import org.lflang.lf.Literal;
import org.lflang.lf.MulExpr;
import org.lflang.lf.ParamRef;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.lf.TupleExpr;
import org.lflang.lf.Type;
import org.lflang.lf.Value;

/**
 * Information about the types of a target language. Contains
 * utilities to convert LF expressions and types to the target
 * language. Each code generator is expected to use at least one
 * language-specific instance of this interface.
 *
 * TODO currently, {@link GeneratorBase} implements this interface,
 *  it should instead contain an instance.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
public interface TargetTypes {


    /**
     * Return true if the target supports generic types
     * (i.e., parametric polymorphism), false otherwise.
     */
    boolean supportsGenerics();


    /**
     * Return the type of time durations.
     */
    String getTargetTimeType();


    /**
     * Return the type of tags.
     */
    String getTargetTagType();


    /**
     * Return the type of fixed sized lists (or arrays).
     */
    String getTargetFixedSizeListType(String baseType, int size);


    /**
     * Return the type of variable sized lists (eg {@code std::vector<baseType>}).
     */
    String getTargetVariableSizeListType(String baseType);


    /**
     * Return an "undefined" type which is used as a default
     * when a type cannot be inferred.
     */
    String getTargetUndefinedType();

    /**
     * Returns a version of the given LF identifier that is
     * escaped properly for insertion into a piece of target
     * code.
     */
    default String escapeIdentifier(String ident) {
        return ident;
    }

    /**
     * Returns an expression in the target language that corresponds
     * to the given time value ({@link #getTargetTimeType()}).
     */
    String getTargetTimeExpr(TimeValue timeValue);

    /**
     * Returns an expression in the target language that is the translation
     * of the given parameter reference. All targets should support this.
     * The default returns the simple name of the parameter.
     */
    default String getTargetParamRef(ParamRef expr, InferredType type) {
        return escapeIdentifier(expr.getParameter().getName());
    }

    /**
     * Returns an expression in the target language that is
     * the translation of the given literal. All targets
     * should support this.
     * The default returns the literal converted to a string.
     */
    default String getTargetLiteral(Literal expr, InferredType type) {
        if (ASTUtils.isZero(expr) && type != null && type.isTime) {
            return getTargetTimeExpr(TimeValue.ZERO);
        }
        return JavaAstUtils.addZeroToLeadingDot(expr.getLiteral()); // unescaped
    }

    /**
     * Returns an expression in the target language that is the translation
     * of the given tuple expression. To support tuple expressions, a target
     * must also register this capability in {@link Target#supportsLfTupleLiterals()}.
     */
    default String getTargetTupleExpr(TupleExpr expr, InferredType type) {
        if (expr.getItems().isEmpty()) {
            return "()";
        }
        InferredType compType = type == null ? null : type.getComponentType();
        return expr.getItems().stream()
                   .map(it -> getTargetExpr(it, compType))
                   .collect(Collectors.joining(", ", "(", ",)"));
    }

    /**
     * Returns an expression in the target language that is the translation
     * of the given list expression. To support list expressions, a target
     * must also register this capability in {@link Target#supportsLfBracketListExpressions()}.
     */
    default String getTargetBracketExpr(BracketExpr expr, InferredType type) {
        InferredType compType = type == null ? null : type.getComponentType();
        return expr.getItems().stream()
                   .map(it -> getTargetExpr(it, compType))
                   .collect(Collectors.joining(", ", "[", "]"));
    }

    /**
     * Returns an expression in the target language that is the translation
     * of the given list expression. To support list expressions, a target
     * must also register this capability in {@link Target#supportsLfBraceListExpressions()}.
     */
    default String getTargetBraceExpr(BraceExpr expr, InferredType type) {
        InferredType compType = type == null ? null : type.getComponentType();
        return expr.getItems().stream()
                   .map(it -> getTargetExpr(it, compType))
                   .collect(Collectors.joining(", ", "{", "}"));
    }

    /**
     * Returns the expression that is used to replace a
     * missing expression in the source language. The expression
     * may for instance be a type-agnostic default value
     * (e.g. Rust's {@code Default::default()}), or produce
     * a compiler error (e.g. Rust's {@code compiler_error!("missing initializer")}).
     *
     * @throws UnsupportedGeneratorFeatureException If the target does not support this
     */
    default String getMissingExpr(InferredType type) {
        throw new UnsupportedGeneratorFeatureException("Missing initializers");
    }


    /**
     * Returns a target type inferred from the type node, or the
     * initializer list. If both are absent, then the undefined
     * type is returned.
     */
    default String getTargetType(Type type, Initializer init) {
        return getTargetType(JavaAstUtils.getInferredType(type, init));
    }

    /**
     * Returns the target type of the type node. This just provides
     * a default parameter for {@link #getTargetType(Type, Initializer)}.
     * If the parameter is null, then the undefined type is returned.
     */
    default String getTargetType(Type type) {
        return getTargetType(type, null);
    }

    /**
     * Return a string representing the specified type in the
     * target language.
     */
    default String getTargetType(InferredType type) {
        if (type.isUndefined()) {
            return getTargetUndefinedType();
        } else if (type.isTime) {
            if (type.isFixedSizeList) {
                return getTargetFixedSizeListType(getTargetTimeType(), type.listSize);
            } else if (type.isVariableSizeList) {
                return getTargetVariableSizeListType(getTargetTimeType());
            } else {
                return getTargetTimeType();
            }
        } else if (type.isFixedSizeList) {
            return getTargetFixedSizeListType(type.baseType(), type.listSize);
        } else if (type.isVariableSizeList) {
            return getTargetVariableSizeListType(type.baseType());
        }
        return type.toText();
    }

    /**
     * Return a string representing the type of the given
     * parameter.
     */
    default String getTargetType(Parameter p) {
        return getTargetType(JavaAstUtils.getInferredType(p));
    }

    /**
     * Return a string representing the type of the given
     * state variable.
     */
    default String getTargetType(StateVar s) {
        return getTargetType(JavaAstUtils.getInferredType(s));
    }

    /**
     * Return a string representing the type of the given
     * action.
     */
    default String getTargetType(Action a) {
        return getTargetType(JavaAstUtils.getInferredType(a));
    }

    /**
     * Return a string representing the type of the given
     * port.
     */
    default String getTargetType(Port p) {
        return getTargetType(JavaAstUtils.getInferredType(p));
    }

    /**
     * Returns the representation of the given initializer
     * expression in target code. The given type, if non-null,
     * may inform the code generation.
     *
     * <p>By default, only initializers with exactly one expression
     * are handled. If you want to provide semantics for other kinds
     * of initializers, for instance, braced initializer lists, override
     * this method.
     *
     * @param init Initializer list (nullable)
     * @param type Declared type of the expression (nullable)
     */
    default String getTargetInitializer(Initializer init, Type type) {
        InferredType inferredType = JavaAstUtils.getInferredType(type, init);
        return getTargetInitializer(init, inferredType);
    }

    default String getTargetInitializer(Initializer init, InferredType inferredType) {
        if (init == null) {
            return getMissingExpr(inferredType);
        }
        Value single = JavaAstUtils.asSingleValue(init);
        return single != null ? getTargetExpr(single, inferredType)
                              : getTargetInitializerWithNotExactlyOneValue(init, inferredType);
    }

    /**
     * Returns the representation of the given initializer if
     * it has not exactly one value. This is an internal implementation
     * detail of {@link #getTargetInitializer(Initializer, Type)},
     * used to support the syntax {@code a(1,2,3)}.
     *
     * @param init Initializer, non-null
     * @param type Inferred type, non-null
     */
    default String getTargetInitializerWithNotExactlyOneValue(Initializer init, InferredType type) {
        // override this method to give semantics to this
        throw new UnsupportedGeneratorFeatureException(init, "Initializers with != 1 value");
    }


    /**
     * Returns the representation of the given value in target code.
     * The given type, if non-null, may inform the code generation.
     */
    default String getTargetExpr(Value value, InferredType type) {
        if (value instanceof ParamRef) {
            return getTargetParamRef((ParamRef) value, type);
        } else if (value instanceof Time) {
            return getTargetTimeExpr((Time) value);
        } else if (value instanceof Literal) {
            return getTargetLiteral((Literal) value, type);
        } else if (value instanceof CodeExpr) {
            Code code = ((CodeExpr) value).getCode();
            if (ASTUtils.isZero(code) && type != null && type.isTime) {
                // todo remove this branch, this is not useful
                return getTargetTimeExpr(TimeValue.ZERO);
            } else {
                return ASTUtils.toText(code);
            }
        } else if (value instanceof TupleExpr) {
            return getTargetTupleExpr((TupleExpr) value, type);
        } else if (value instanceof BracketExpr) {
            return getTargetBracketExpr((BracketExpr) value, type);
        } else if (value instanceof BraceExpr) {
            return getTargetBraceExpr((BraceExpr) value, type);
        } else if (value instanceof MulExpr) {
            MulExpr e = (MulExpr) value;
            return getTargetExpr(e.getLeft(), null)
                + " " + e.getOp()
                + " " + getTargetExpr(e.getRight(), null);
        } else if (value instanceof AddExpr) {
            AddExpr e = (AddExpr) value;
            return getTargetExpr(e.getLeft(), null)
                + " " + e.getOp()
                + " " + getTargetExpr(e.getRight(), null);
        } else {
            throw new IllegalStateException("Invalid value " + value);
        }
    }

    /**
     * Returns the representation of the given time value in
     * target code.
     */
    default String getTargetTimeExpr(Time t) {
        return getTargetTimeExpr(JavaAstUtils.toTimeValue(t));
    }


}
