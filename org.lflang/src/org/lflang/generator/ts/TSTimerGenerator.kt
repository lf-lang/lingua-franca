package org.lflang.generator.ts

import org.lflang.lf.Expression
import org.lflang.lf.Timer
import java.util.*

/**
 * Generator timers for TypeScript target.
 */
class TSTimerGenerator (
    // TODO(hokeun): Remove dependency on TSGenerator.
    private val tsGenerator: TSGenerator,
    private val timers: List<Timer>
) {
    private fun Expression.getTargetValue(): String = tsGenerator.getTargetValueW(this)

    fun generateClassProperties(): String {
        val timerClassProperties = LinkedList<String>()
        for (timer in timers) {
            timerClassProperties.add("${timer.name}: __Timer;")
        }
        return timerClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val timerInstantiations = LinkedList<String>()
        for (timer in timers) {
            val timerPeriod: String = timer.period?.getTargetValue() ?: "0"
            val timerOffset: String = timer.offset?.getTargetValue() ?: "0"

            timerInstantiations.add("this.${timer.name} = new __Timer(this, $timerOffset, $timerPeriod);")
        }
        return timerInstantiations.joinToString("\n")
    }
}
