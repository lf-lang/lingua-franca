package org.lflang.generator.ts;

import org.lflang.generator.getTargetTimeExpr
import org.lflang.generator.orZero
import org.lflang.lf.Timer
import java.util.*

/**
 * Generator timers for TypeScript target.
 */
class TSTimerGenerator (
    private val timers: List<Timer>
) {

    fun generateClassProperties(): String {
        val timerClassProperties = mutableListOf<String>()
        for (timer in timers) {
            timerClassProperties.add("${timer.name}: __Timer;")
        }
        return timerClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val timerInstantiations = mutableListOf<String>()
        for (timer in timers) {
            val timerPeriod: String = TsTypes.getTargetTimeExpr(timer.period.orZero())
            val timerOffset: String = TsTypes.getTargetTimeExpr(timer.offset.orZero())

            timerInstantiations.add("this.${timer.name} = new __Timer(this, $timerOffset, $timerPeriod);")
        }
        return timerInstantiations.joinToString("\n")
    }
}
