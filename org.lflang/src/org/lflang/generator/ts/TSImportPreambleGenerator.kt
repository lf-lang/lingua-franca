/*************
 * Copyright (c) 2019-2020, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.ts

import org.lflang.generator.PrependOperator
import java.nio.file.Path
import java.util.*

/**
 * Preamble generator for imports in TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */

class TSImportPreambleGenerator(
    private val filePath: Path,
    private val protoFiles: MutableList<String>
) {
    companion object {
        /**
         * Default imports for importing all the core classes and helper classes
         * for CLI argument handling.
         */
        const val DEFAULT_IMPORTS =  """
            |import commandLineArgs from 'command-line-args'
            |import commandLineUsage from 'command-line-usage'
            |import {Parameter as __Parameter, Timer as __Timer, Reactor as __Reactor, App as __App} from './core/reactor'
            |import {Action as __Action, Startup as __Startup, FederatePortAction as __FederatePortAction} from './core/action'
            |import {Bank as __Bank} from './core/bank'
            |import {FederatedApp as __FederatedApp} from './core/federation'
            |import {InPort as __InPort, OutPort as __OutPort, Port as __Port, WritablePort as __WritablePort, WritableMultiPort as __WritableMultiPort} from './core/port'
            |import {InMultiPort as __InMultiPort, OutMultiPort as __OutMultiPort} from './core/multiport'
            |import {Reaction as __Reaction} from './core/reaction'
            |import {State as __State} from './core/state'
            |import {TimeUnit, TimeValue, Tag as __Tag, Origin as __Origin} from './core/time'
            |import {Args as __Args, Variable as __Variable, Triggers as __Triggers, Present, Read, Write, ReadWrite, MultiReadWrite, Sched} from './core/types'
            |import {Log} from './core/util'
            |import {ProcessedCommandLineArgs as __ProcessedCommandLineArgs, CommandLineOptionDefs as __CommandLineOptionDefs, CommandLineUsageDefs as __CommandLineUsageDefs, CommandLineOptionSpec as __CommandLineOptionSpec, unitBasedTimeValueCLAType as __unitBasedTimeValueCLAType, booleanCLAType as __booleanCLAType} from './core/cli'
            |import {Parameter as __Parameter, Timer as __Timer, Reactor as __Reactor, App as __App} from 'reactor-ts'
            |import {Action as __Action, Startup as __Startup, FederatePortAction as __FederatePortAction} from 'reactor-ts'
            |import {Bank as __Bank} from 'reactor-ts'
            |import {FederatedApp as __FederatedApp} from 'reactor-ts'
            |import {InPort as __InPort, OutPort as __OutPort, Port as __Port} from 'reactor-ts'
            |import {InMultiPort as __InMultiPort, OutMultiPort as __OutMultiPort} from 'reactor-ts'
            |import {Reaction as __Reaction} from 'reactor-ts'
            |import {State as __State} from 'reactor-ts'
            |import {TimeUnit, TimeValue, Tag as __Tag, Origin as __Origin} from 'reactor-ts'
            |import {Args as __Args, Variable as __Variable, Triggers as __Triggers, Present, Read, Write, ReadWrite, MultiReadWrite, Sched} from 'reactor-ts'
            |import {Log} from 'reactor-ts'
            |import {ProcessedCommandLineArgs as __ProcessedCommandLineArgs, CommandLineOptionDefs as __CommandLineOptionDefs, CommandLineUsageDefs as __CommandLineUsageDefs, CommandLineOptionSpec as __CommandLineOptionSpec, unitBasedTimeValueCLAType as __unitBasedTimeValueCLAType, booleanCLAType as __booleanCLAType} from 'reactor-ts'
            |"""
    }

    private fun generateDefaultImports(): String {
        return with(PrependOperator) {
            """
                |// Code generated by the Lingua Franca compiler from:
                |// file:/${filePath.toString()}
                $DEFAULT_IMPORTS
                |
            """.trimMargin()
        }
    }

    private fun generateProtoPreamble(): String {
        val protoFileImports = StringJoiner("\n")
        for (file in protoFiles) {
            var name = file
            // Remove any extension the file name may have.
            val dot = name.lastIndexOf('.')
            if (dot > 0) {
                name = name.substring(0, dot)
            }
            protoFileImports.add("""
                |import * as ${name} from "./${name}_pb"
            """.trimMargin())
        }
        return with(PrependOperator) {"""
                |// Imports for protocol buffers
                |${protoFileImports}
                |
                |
            """.trimMargin()
        }
    }

    fun generatePreamble(): String {
        return generateDefaultImports() + generateProtoPreamble()
    }
}