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

import org.lflang.Target
import org.lflang.lf.Action
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef

/**
 *
 */
class CppGenerator2 : KtGeneratorBase(
    Target.CPP,
    supportsGenerics = true
) {


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

    override fun generateDelayBody(action: Action, port: VarRef): String? {
        TODO("Not yet implemented")
    }

    override fun generateForwardBody(action: Action, port: VarRef): String? {
        TODO("Not yet implemented")
    }

    override fun generateDelayGeneric(): String? {
        TODO("Not yet implemented")
    }

    override fun getTargetTimeType(): String {
        TODO("Not yet implemented")
    }
    override val targetTagType: String
        get() = TODO("Not yet implemented")
    override val targetTagIntervalType: String
        get() = TODO("Not yet implemented")
    override val targetUndefinedType: String
        get() = TODO("Not yet implemented")

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String {
        TODO("Not yet implemented")
    }

    override fun getTargetVariableSizeListType(baseType: String): String {
        TODO("Not yet implemented")
    }

}
