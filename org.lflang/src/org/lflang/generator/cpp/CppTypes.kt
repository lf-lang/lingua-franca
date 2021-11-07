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

package org.lflang.generator.cpp

import org.lflang.InferredType
import org.lflang.generator.TargetTypes
import org.lflang.lf.BraceExpr
import org.lflang.lf.Initializer
import org.lflang.lf.ParamRef
import org.lflang.lf.TimeUnit

/**
 * Implementation of [TargetTypes] for C++.
 *
 * @author Clément Fournier
 */
object CppTypes : TargetTypes {

    override fun supportsGenerics() = true

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = "std::array<$baseType, $size>"
    override fun getTargetVariableSizeListType(baseType: String) = "std::vector<$baseType>"

    override fun getTargetInitializer(init: Initializer?, inferredType: InferredType): String {
        return getCppInitializerWithTypePrefix(init, inferredType)
    }

    override fun getTargetBraceExpr(expr: BraceExpr, type: InferredType?): String {
        val braceInitializer = super.getTargetBraceExpr(expr, type)
        return if (type?.isList == true) {
            // prefix the {...} with std::vector<_>
            if (type.isTime)
                getTargetVariableSizeListType(targetTimeType) + braceInitializer
            else
                getTargetVariableSizeListType(type.baseType()) + braceInitializer
        } else {
            braceInitializer
        }
    }

    override fun getTargetUndefinedType() = "void"

    override fun getTargetTimeExpr(magnitude: Long, unit: TimeUnit): String =
        if (magnitude == 0L) "reactor::Duration::zero()"
        else magnitude.toString() + unit.cppUnit

}

fun CppTypes.getCppInitializerWithTypePrefix(init: Initializer?, type: InferredType): String =
    if (init == null) getMissingExpr(type)
    else getTargetType(type) + getCppInitializerWithoutTypePrefix(init, type)

fun CppTypes.getCppInitializerWithoutTypePrefix(init: Initializer?, inferredType: InferredType?): String {
    if (init == null) {
        return getMissingExpr(inferredType)
    }
    val singleExpr = init.exprs.singleOrNull()
    return if (init.isAssign && singleExpr is BraceExpr)
        singleExpr.items.joinToString(", ", "{", "}") {
            getTargetExpr(it, inferredType?.componentType)
        }
    else buildString {
        if (init.isAssign) {
            val expr = init.exprs.single()
            append("(").append(getTargetExpr(expr, inferredType)).append(")")
        } else {
            val (prefix, postfix) = if (init.isBraces) Pair("{", "}") else Pair("(", ")")
            init.exprs.joinTo(this, ", ", prefix, postfix) {
                getTargetExpr(it, inferredType?.componentType)
            }
        }
    }
}

/**
 * This object generates types in the context of the outer class,
 * where parameter references need special handling.
 */
object CppOuterTypes : TargetTypes by CppTypes {

    override fun getTargetParamRef(expr: ParamRef, type: InferredType?): String {
        return "__lf_inner.${expr.parameter.name}"
    }

}

/** Get a C++ representation of a LF unit. */
val TimeUnit.cppUnit
    get() = when (this) {
        TimeUnit.NSEC    -> "ns"
        TimeUnit.NSECS   -> "ns"
        TimeUnit.USEC    -> "us"
        TimeUnit.USECS   -> "us"
        TimeUnit.MSEC    -> "ms"
        TimeUnit.MSECS   -> "ms"
        TimeUnit.SEC     -> "s"
        TimeUnit.SECS    -> "s"
        TimeUnit.SECOND  -> "s"
        TimeUnit.SECONDS -> "s"
        TimeUnit.MIN     -> "min"
        TimeUnit.MINS    -> "min"
        TimeUnit.MINUTE  -> "min"
        TimeUnit.MINUTES -> "min"
        TimeUnit.HOUR    -> "h"
        TimeUnit.HOURS   -> "h"
        TimeUnit.DAY     -> "d"
        TimeUnit.DAYS    -> "d"
        TimeUnit.WEEK    -> "d*7"
        TimeUnit.WEEKS   -> "d*7"
        else             -> ""
    }
