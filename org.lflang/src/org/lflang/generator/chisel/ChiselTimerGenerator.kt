/*************
 * Copyright (c) 2021, TU Dresden.

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

package org.lflang.generator.chisel

import org.lflang.TimeValue
import org.lflang.generator.orZero
import org.lflang.lf.Reactor
import org.lflang.lf.Timer
import org.lflang.lf.Time


/** A C++ code generator for timers */
class ChiselTimerGenerator(private val reactor: Reactor) {

//    private fun generateInitializer(timer: Timer): String {
//        val offset = timer.offset.orZero().toCppTime()
//        val period = timer.period.orZero().toCppTime()
//        return """${timer.name}{"${timer.name}", this, $period, $offset}"""
//    }
//
//    /** Get all timer declarations */

    fun generateTimerConfig(timer: Timer): String {
        val offset = ChiselTypes.getTargetTimeExpr(timer.offset.orZero())
        val period = ChiselTypes.getTargetTimeExpr(timer.period.orZero())

        return "TimerConfig(offset = $offset, period = $period)"
    }
    fun generateDeclarations() =
        reactor.timers.joinToString(separator = "\n", prefix = "// timers\n", postfix = "\n") {
            "val ${it.name} = Module(new Timer(${generateTimerConfig(it)}));"
        }
//
//    /** Get all timer initializers */
//    fun generateInitializers() =
//        reactor.timers.joinToString(separator = "\n", prefix = "// timers\n") { ", ${generateInitializer(it)}" }
}
