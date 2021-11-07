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

package org.lflang.generator.ts

import org.lflang.InferredType
import org.lflang.generator.TargetTypes
import org.lflang.inferredType
import org.lflang.lf.ParamRef
import org.lflang.lf.Parameter
import org.lflang.lf.StateVar
import org.lflang.lf.TimeUnit

/**
 * [TargetTypes] implementation for [TSGenerator].
 *
 * @author Clément Fournier
 * @author Hokeun Kim
 */
object TsTypes : TargetTypes {
    override fun supportsGenerics(): Boolean = true

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


    override fun getTargetTimeExpr(magnitude: Long, unit: TimeUnit): String =
        if (unit != TimeUnit.NONE) "TimeValue.$unit($magnitude)"
        // The value must be zero.
        else "TimeValue.zero()"

    override fun getTargetParamRef(expr: ParamRef, type: InferredType?): String =
        "this.${expr.parameter.name}.get()"


    /**
     * Returns a type for the state variable. Note that this is TS-specific
     * and not equivalent to `s.type.targetType`.
     */
    fun getTargetType(s: StateVar): String {
        val type = getTargetType(s.inferredType)
        return if (s.init == null) {
            "$type | undefined"
        } else {
            type
        }
    }
}
