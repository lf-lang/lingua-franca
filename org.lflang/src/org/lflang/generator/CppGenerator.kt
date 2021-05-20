package org.lflang.generator

import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.Action
import org.lflang.lf.VarRef

class CppGenerator: GeneratorBase() {
    override fun generateDelayBody(action: Action, port: VarRef) = null // TODO
    override fun generateForwardBody(action: Action, port: VarRef) = null // TODO
    override fun generateDelayGeneric() = null // TODO
    override fun supportsGenerics() = true // TODO

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetTagIntervalType() = getTargetUndefinedType()

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = null // TODO
    override fun  getTargetVariableSizeListType(baseType: String) = null // TODO

    override fun getTargetUndefinedType() = null // TODO

    override fun getTarget() = Target.CPP
}