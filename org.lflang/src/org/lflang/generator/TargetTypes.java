package org.lflang.generator;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.JavaAstUtils;
import org.lflang.TimeValue;
import org.lflang.lf.Time;
import org.lflang.lf.TimeUnit;
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
 */
public interface TargetTypes {


    /**
     * Return true if the target supports generics (i.e., parametric
     * polymorphism), false otherwise.
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
     * to a time value ({@link #getTargetTimeType()}), with the given
     * magnitude and unit. The unit may not be null (use {@link TimeUnit#NONE}).
     */
    default String getTargetTimeExpression(long magnitude, TimeUnit unit) {
        // todo make non-default when we reuse this for all generators,
        //  all targets should support this.
        Objects.requireNonNull(unit);
        throw new UnsupportedGeneratorFeatureException("Time expressions");
    }

    /**
     * Returns an expression in the target language that corresponds
     * to a variable-size list expression.
     *
     * @throws UnsupportedGeneratorFeatureException If the target does not support this
     */
    default String getVariableSizeListInitExpression(List<String> contents, boolean withBraces) {
        throw new UnsupportedGeneratorFeatureException("Variable size lists");
    }

    /**
     * Returns an expression in the target language that corresponds
     * to a fixed-size list expression.
     *
     * @throws UnsupportedGeneratorFeatureException If the target does not support this
     */
    default String getFixedSizeListInitExpression(List<String> contents, int listSize, boolean withBraces) {
        throw new UnsupportedGeneratorFeatureException("Fixed size lists");
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
    default String getTargetType(Type type, List<Value> init) {
        return getTargetType(JavaAstUtils.getInferredType(type, init));
    }

    /**
     * Returns the target type of the type node. This just provides
     * a default parameter for {@link #getTargetType(Type, List)}.
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
     * Returns the representation of the given initializer
     * expression in target code. The given type, if non-null,
     * may inform the code generation.
     *
     * @param init           Initializer list (non-null)
     * @param type           Declared type of the expression (nullable)
     * @param initWithBraces Whether the initializer uses the braced form.
     */
    default String getTargetInitializer(List<Value> init, Type type, boolean initWithBraces) {
        Objects.requireNonNull(init);
        var inferredType = JavaAstUtils.getInferredType(type, init);
        if (init.size() == 1) {
            return getTargetExpr(init.get(0), inferredType);
        }
        var targetValues = init.stream().map(it -> getTargetExpr(it, inferredType)).collect(Collectors.toList());
        if (inferredType.isFixedSizeList) {
            return getFixedSizeListInitExpression(targetValues, inferredType.listSize, initWithBraces);
        } else if (inferredType.isVariableSizeList) {
            return getVariableSizeListInitExpression(targetValues, initWithBraces);
        } else {
            return getMissingExpr(inferredType);
        }
    }


    /**
     * Returns the representation of the given value in target code.
     * The given type, if non-null, may inform the code generation.
     */
    default String getTargetExpr(Value value, InferredType type) {
        if (ASTUtils.isZero(value) && type != null && type.isTime) {
            return getTargetTimeExpression(0, TimeUnit.NONE);
        } else if (value.getParameter() != null) {
            return escapeIdentifier(value.getParameter().getName());
        } else if (value.getTime() != null) {
            return getTargetTimeExpr(value.getTime());
        } else if (value.getLiteral() != null) {
            return JavaAstUtils.addZeroToLeadingDot(value.getLiteral()); // here we don't escape
        } else if (value.getCode() != null) {
            return ASTUtils.toText(value.getCode());
        } else {
            throw new IllegalStateException("Invalid value " + value);
        }
    }


    /**
     * Returns the representation of the given time value in
     * target code.
     */
    default String getTargetTimeExpr(TimeValue tv) {
        return getTargetTimeExpression(tv.time, tv.unit);
    }

    /**
     * Returns the representation of the given time value in
     * target code.
     */
    default String getTargetTimeExpr(Time t) {
        return getTargetTimeExpression(t.getInterval(), t.getUnit());
    }
}
