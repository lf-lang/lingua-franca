/*
 * Copyright (c) 2022, TU Dresden.
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

package org.lflang.generator.uc

import org.lflang.InferredType
import org.lflang.TimeUnit
import org.lflang.TimeValue
import org.lflang.generator.TargetTypes
import org.lflang.lf.Initializer
import org.lflang.lf.ParameterReference
import org.lflang.lf.ParenthesisListExpression

object UcTypes : TargetTypes {

    override fun supportsGenerics() = true

    override fun getTargetTimeType() = "interval_t"
    override fun getTargetTagType() = "tag_t"
    override fun getTargetUndefinedType() = "void"

    override fun getTargetTimeExpr(timeValue: TimeValue): String =
        with(timeValue) {
            if (magnitude == 0L) "0"
            else "${unit.cUnit}(${magnitude.toString()})"
        }
}

val TimeUnit?.cUnit
    get() = when (this) {
        TimeUnit.NANO   -> "NSEC"
        TimeUnit.MICRO  -> "USEC"
        TimeUnit.MILLI  -> "MSEC"
        TimeUnit.SECOND -> "SEC"
        TimeUnit.MINUTE -> "MIN"
        TimeUnit.HOUR   -> "HOUR"
        TimeUnit.DAY    -> "DAY"
        TimeUnit.WEEK   -> "WEEK"
        else            -> ""
    }
