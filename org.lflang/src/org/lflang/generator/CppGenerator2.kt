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
class CppGenerator2 : GeneratorBase() {


    /**
     * Return a string representing the specified type in the target language.
     * @param type The type.
     */
    private fun getTargetType(type: InferredType, undefinedType ): String {
        val _isUndefined = type.isUndefined
        if (_isUndefined) {
            return this.getTargetUndefinedType()
        } else {
            if (type.isTime.toBoolean()) {
                return if (type.isFixedSizeList.toBoolean()) {
                    this.getTargetFixedSizeListType(this.getTargetTimeType(), type.listSize)
                } else {
                    if (type.isVariableSizeList.toBoolean()) {
                        this.getTargetVariableSizeListType(this.getTargetTimeType())
                    } else {
                        this.getTargetTimeType()
                    }
                }
            } else {
                if (type.isFixedSizeList.toBoolean()) {
                    return this.getTargetFixedSizeListType(type.baseType(), type.listSize)
                } else {
                    if (type.isVariableSizeList.toBoolean()) {
                        return this.getTargetVariableSizeListType(type.baseType())
                    }
                }
            }
        }
        return type.toText()
    }

    fun StateVar.getTargetType(): String = getTargetType(ASTUtils.getInferredType(this))
    fun Action.getTargetType(): String = getTargetType(ASTUtils.getInferredType(this))
    fun Port.getTargetType(): String = getTargetType(ASTUtils.getInferredType(this))
    fun Type.getTargetType(): String = getTargetType(ASTUtils.getInferredType(this))
    fun Parameter.getTargetType(): String = getTargetType(ASTUtils.getInferredType(this))



/*

    def declareParameters(Reactor r) '''
        «FOR p : r.parameters BEFORE '// parameters\n' AFTER '\n'»
            std::add_const<«p.targetType»>::type «p.name»;
        «ENDFOR»
    '''
 */

    fun Reactor.declareParameters() =
        parameters.joinToString(prefix = "// parameters\n", postfix = "\n") {
            "std::add_const<${it.targetType}>::type ${it.name};"
        }


    override fun generateDelayBody(action: Action?, port: VarRef?): String {
        TODO("Not yet implemented")
    }

    override fun generateForwardBody(action: Action?, port: VarRef?): String {
        TODO("Not yet implemented")
    }

    override fun generateDelayGeneric(): String {
        TODO("Not yet implemented")
    }

    override fun supportsGenerics(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getTargetTimeType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetTagType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetTagIntervalType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetUndefinedType(): String {
        TODO("Not yet implemented")
    }

    override fun getTargetFixedSizeListType(baseType: String?, size: Int?): String {
        TODO("Not yet implemented")
    }

    override fun getTargetVariableSizeListType(baseType: String?): String {
        TODO("Not yet implemented")
    }

    override fun getTarget(): Target {
        TODO("Not yet implemented")
    }

}
