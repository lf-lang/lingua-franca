package org.lflang.generator;

import java.util.List;

import org.lflang.InferredType;
import org.lflang.lf.TimeUnit;

/**
 * Information about the types of a target language.
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
        throw new UnsupportedOperationException();
    }


    default String getVariableSizeListInitExpression(List<String> contents, boolean withBraces) {
        throw new UnsupportedOperationException();
    }


    default String getFixedSizeListInitExpression(List<String> contents, boolean withBraces) {
        throw new UnsupportedOperationException();
    }


    default String getMissingExpr(InferredType type) {
        throw new UnsupportedOperationException();
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
}
