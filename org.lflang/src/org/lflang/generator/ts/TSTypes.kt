package org.lflang.generator.ts

import org.lflang.InferredType
import org.lflang.TimeValue
import org.lflang.generator.TargetTypes
import org.lflang.inferredType
import org.lflang.lf.Initializer
import org.lflang.lf.ParamRef
import org.lflang.lf.StateVar

/**
 * [TargetTypes] implementation for [TSGenerator].
 *
 * @author Peter Donovan
 * @author Clément Fournier
 * @author Hokeun Kim
 */

object TSTypes : TargetTypes {

    /**
     * Returns a type for the state variable. Note that this is TS-specific
     * and not equivalent to `s.type.targetType`.
     */
    override fun getTargetType(s: StateVar): String {
        val type = getTargetType(s.inferredType)
        return if (s.init == null) {
            "$type | undefined"
        } else {
            type
        }
    }

    override fun supportsGenerics(): Boolean {
        return true
    }

    override fun getTargetTimeType(): String {
        return "TimeValue"
    }

    override fun getTargetTagType(): String {
        return "TimeValue"
    }

    override fun getTargetUndefinedType(): String {
        return "Present"
    }

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String {
        return "Array($size)<$baseType>"
    }

    override fun getTargetVariableSizeListType(baseType: String): String {
        return "Array<$baseType>"
    }


    override fun getTargetTimeExpr(value: TimeValue): String =
        with(value) {
            if (unit != null) "TimeValue.${unit.canonicalName}($magnitude)"
            // The value must be zero.
            else "TimeValue.zero()"
        }

    override fun getTargetParamRef(expr: ParamRef, type: InferredType?): String =
        "this.${expr.parameter.name}.get()"

    override fun getTargetInitializerWithNotExactlyOneValue(init: Initializer, type: InferredType): String =
        init.exprs.joinToString(", ", "[", "]") {
            getTargetExpr(it, type.componentType)
        }

}
