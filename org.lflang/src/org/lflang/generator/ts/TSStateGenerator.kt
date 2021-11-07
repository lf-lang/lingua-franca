package org.lflang.generator.ts

import org.lflang.generator.getTargetInitializer
import org.lflang.lf.StateVar
import java.util.*

/**
 * Generator for state variables in TypeScript target.
 */
class TSStateGenerator(
    private val stateVars: List<StateVar>
) {

    fun generateClassProperties(): String =
        stateVars.joinToString("\n") {
            "${it.name}: __State<${TsTypes.getTargetType(it)}>;"
        }

    fun generateInstantiations(): String =
        stateVars.joinToString("\n") {
            val init = if (it.init != null) TsTypes.getTargetInitializer(it) else "undefined"
            "this.${it.name} = new __State($init);"
        }
}
