/* Generator for Cpp target. */

/*************
 * Copyright (c) 2019-2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.Action
import org.lflang.lf.VarRef

class CppGenerator : GeneratorBase() {

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGeneratre() in GeneratorBase
        if (generatorErrorsOccurred) return;
    }

    override fun generateDelayBody(action: Action, port: VarRef) = null // TODO
    override fun generateForwardBody(action: Action, port: VarRef) = null // TODO
    override fun generateDelayGeneric() = null // TODO
    override fun supportsGenerics() = true // TODO

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetTagIntervalType() = getTargetUndefinedType()

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = null // TODO
    override fun getTargetVariableSizeListType(baseType: String) = null // TODO

    override fun getTargetUndefinedType() = null // TODO

    override fun getTarget() = Target.CPP
}