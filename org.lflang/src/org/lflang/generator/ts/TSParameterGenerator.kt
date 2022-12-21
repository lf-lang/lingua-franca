package org.lflang.generator.ts

import org.lflang.joinWithLn
import org.lflang.lf.Parameter

/**
 * Generate parameters for TypeScript target.
 */
class TSParameterGenerator(
    private val parameters: List<Parameter>
 ) {

    fun generateClassProperties(): String =
        parameters.joinWithLn {
            "${it.name}: __Parameter<${TSTypes.getTargetType(it)}>;"
        }

    fun generateInstantiations(): String =
        parameters.joinWithLn {
            "this.${it.name} = new __Parameter(${it.name});"
        }
}
