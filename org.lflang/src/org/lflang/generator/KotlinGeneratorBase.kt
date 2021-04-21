/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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

package org.lflang.generator

import org.lflang.ASTUtils
import org.lflang.InferredType
import org.lflang.Target
import org.lflang.lf.*

/**
 *
 */
abstract class KotlinGeneratorBase(val target: Target) {

    abstract fun generateDelayBody(action: Action, port: VarRef): String
    abstract fun generateForwardBody(action: Action, port: VarRef): String
    abstract fun generateDelayGeneric(): String
    abstract fun supportsGenerics(): Boolean
    abstract fun getTargetTimeType(): String
    abstract fun getTargetTagType(): String
    abstract fun getTargetTagIntervalType(): String
    abstract fun getTargetUndefinedType(): String
    abstract fun getTargetFixedSizeListType(baseType: String, size: Int): String
    abstract fun getTargetVariableSizeListType(baseType: String): String


    /** Return a string representing the specified type in the target language. */
    protected fun getTargetType(type: InferredType): String = when {
        type.isUndefined        -> this.getTargetUndefinedType()
        type.isTime             -> when {
            type.isFixedSizeList    -> this.getTargetFixedSizeListType(this.getTargetTimeType(), type.listSize)
            type.isVariableSizeList -> this.getTargetVariableSizeListType(this.getTargetTimeType())
            else                    -> this.getTargetTimeType()
        }
        type.isFixedSizeList    -> this.getTargetFixedSizeListType(type.baseType(), type.listSize)
        type.isVariableSizeList -> this.getTargetVariableSizeListType(type.baseType())
        else                    -> type.toText()
    }


    val StateVar.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Action.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Port.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Type.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
    val Parameter.targetType: String get() = getTargetType(ASTUtils.getInferredType(this))
}


class Foo {

}
