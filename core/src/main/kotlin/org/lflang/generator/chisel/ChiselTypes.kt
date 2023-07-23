/**
 * @author Erling R. Jellum (erling.r.jellum@ntnu.no)
 *
 * Copyright (c) 2023, The Norwegian University of Science and Technology.
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

package org.lflang.generator.chisel

import org.lflang.ast.ASTUtils
import org.lflang.generator.TargetTypes
import org.lflang.generator.cpp.toCppTime
import org.lflang.lf.Time
import org.lflang.lf.Literal
import org.lflang.lf.Expression

enum class ChiselTokenType {
    Single, Array, Fifo
}


object ChiselTypes : TargetTypes {
    fun getTargetTimeExpr(e: Expression): String {
        if (e is Literal)
            return "Time.nsec(${e.literal})"
        else if (e is Time)
            return "Time.nsec(${ASTUtils.toTimeValue(e).toNanoSeconds().toString()})"
        else
            throw Error("Error")
    }
    override fun supportsGenerics(): Boolean {
        TODO("Not yet implemented")
    }
    override fun getTargetTimeType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetFixedSizeListType(baseType: String?, size: Int): String {
        TODO("Not yet implemented")
    }

    override fun getTargetTagType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetUndefinedType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetVariableSizeListType(baseType: String?): String {
        TODO("Not yet implemented")
    }
}