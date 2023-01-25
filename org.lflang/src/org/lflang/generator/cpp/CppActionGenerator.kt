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

package org.lflang.generator.cpp

import org.lflang.ErrorReporter
import org.lflang.generator.PrependOperator
import org.lflang.generator.orZero
import org.lflang.inferredType
import org.lflang.isLogical
import org.lflang.lf.Action
import org.lflang.lf.BuiltinTrigger
import org.lflang.lf.Reactor

/** A C++ code generator for actions */
class CppActionGenerator(private val reactor: Reactor, private val errorReporter: ErrorReporter) {

    companion object {
        val Action.cppType: String
            get() {
                val dataType = inferredType.cppType
                return if (this.isLogical) "reactor::LogicalAction<$dataType>"
                else "reactor::PhysicalAction<$dataType>"
            }

        val startupName: String = BuiltinTrigger.STARTUP.literal
        val shutdownName: String = BuiltinTrigger.SHUTDOWN.literal
    }

    private fun generateDeclaration(action: Action) = "${action.cppType} ${action.name};"

    private fun generateInitializer(action: Action) =
        if (action.isLogical) generateLogicalInitializer(action) else initializePhysicalInitializer(action)

    private fun generateLogicalInitializer(action: Action): String {
        return if (action.minSpacing != null || !action.policy.isNullOrEmpty()) {
            errorReporter.reportError(
                action,
                "minSpacing and spacing violation policies are not yet supported for logical actions in reactor-ccp!"
            )
        } else {
            val time = action.minDelay.orZero().toCppTime()
            """, ${action.name}{"${action.name}", this, $time}"""
        }
    }

    private fun initializePhysicalInitializer(action: Action): String {
        return if (action.minDelay != null || action.minSpacing != null || !action.policy.isNullOrEmpty()) {
            errorReporter.reportError(
                action,
                "minDelay, minSpacing and spacing violation policies are not yet supported for physical actions in reactor-ccp!"
            )
        } else {
            """, ${action.name}{"${action.name}", this}"""
        }
    }

    /** Get all action declarations */
    fun generateDeclarations() = with(PrependOperator) {
        """
        ${
            " |"..reactor.actions.joinToString("\n", "// actions\n", "\n") { generateDeclaration(it) }
        }
            |// default actions
            |reactor::StartupTrigger $startupName {"$startupName", this};
            |reactor::ShutdownTrigger $shutdownName {"$shutdownName", this};
        """.trimMargin()
    }

    /** Get all action initializers */
    fun generateInitializers() =
        reactor.actions.joinToString(separator = "\n", prefix = "// actions\n", postfix = "\n") { generateInitializer(it) }
}
