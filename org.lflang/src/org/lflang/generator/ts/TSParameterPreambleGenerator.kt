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

import org.lflang.FileConfig
import org.lflang.TargetConfig
import org.lflang.joinWithCommasLn
import org.lflang.joinWithLn
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import java.util.StringJoiner

/**
 * Preamble generator for global parameters in TypeScript target.
 *
 *  @author Matt Weber
 *  @author Edward A. Lee
 *  @author Marten Lohstroh
 *  @author Christian Menard
 *  @author Hokeun Kim
 */

class TSParameterPreambleGenerator(
    private val fileConfig: FileConfig,
    private val targetConfig: TargetConfig,
    private val reactors: MutableList<Reactor>
) {

    private fun getTimeoutTimeValue(): String =
        targetConfig.timeout?.toTsTime() ?: "undefined"

    private fun getParameters(): List<Parameter> {
        var mainReactor: Reactor? = null
        for (reactor in reactors) {
            if (reactor.isMain || reactor.isFederated) {
                mainReactor = reactor
            }
        }
        return mainReactor?.parameters ?: emptyList()
    }

    /**
     * Assign results of parsing custom command line arguments
     */
    private fun assignCustomCLArgs(mainParameters: HashSet<Parameter>): String =
        mainParameters.joinWithLn { parameter ->

            """
                |let __CL${parameter.name}: ${TSTypes.getTargetType(parameter)} | undefined = undefined;
                |if (__processedCLArgs.${parameter.name} !== undefined) {
                |    if (__processedCLArgs.${parameter.name} !== null) {
                |        __CL${parameter.name} = __processedCLArgs.${parameter.name};
                |    } else {
                |        Log.global.error(__clUsage);
                |        throw new Error("Custom '${parameter.name}' command line argument is malformed.");
                |    }
                |}
                """
        }

    /**
     * Generate code for extracting custom command line arguments
     * from the object returned from commandLineArgs
     */
    private fun logCustomCLArgs(mainParameters: Set<Parameter>): String =
        mainParameters.joinWithLn { parameter ->
            // We can't allow the programmer's parameter names
            // to cause the generation of variables with a "__" prefix
            // because they could collide with other variables.
            // So prefix variables created here with __CL
            """
                |if (__processedCLArgs.${parameter.name} !== undefined && __processedCLArgs.${parameter.name} !== null
                |    && !__noStart) {
                |    Log.global.info("'${parameter.name}' property overridden by command line argument.");
                |}"""
        }

    fun generateParameters(): Pair<Set<Parameter>, String> {
        /**
         * Set of parameters (AST elements) associated with the main reactor.
         */
        val mainParameters = HashSet<Parameter>()

        // Build the argument spec for commandLineArgs and commandLineUsage
        val customArgs = StringJoiner(",\n")

        // Extend the return type for commandLineArgs
        val clTypeExtension = StringJoiner(", ")

        for (parameter in getParameters()) {
            var customArgType: String? = null
            var customTypeLabel: String? = null

            val paramType = TSTypes.getTargetType(parameter)
            if (paramType == "string") {
                mainParameters.add(parameter)
                customArgType = "String";
            } else if (paramType == "number") {
                mainParameters.add(parameter)
                customArgType = "Number";
            } else if (paramType == "boolean") {
                mainParameters.add(parameter)
                customArgType = "booleanCLAType";
                customTypeLabel = "[true | false]"
            } else if (paramType == "TimeValue") {
                mainParameters.add(parameter)
                customArgType = "__unitBasedTimeValueCLAType"
                customTypeLabel = "\'<duration> <units>\'"
            }
            // Otherwise don't add the parameter to customCLArgs

            if (customArgType != null) {
                clTypeExtension.add(parameter.name + ": " + paramType)
                if (customTypeLabel != null) {
                    customArgs.add(
                        """
                    |{
                    |    name: '${parameter.name}',
                    |    type: $customArgType,
                    |    typeLabel: "{underline $customTypeLabel}",
                    |    description: 'Custom argument. Refer to ${fileConfig.srcFile} for documentation.'
                    |}""".trimMargin()
                    )
                } else {
                    customArgs.add(
                        """
                    |{
                    |    name: '${parameter.name}',
                    |    type: $customArgType,
                    |    description: 'Custom argument. Refer to ${fileConfig.srcFile} for documentation.'
                    |}""".trimMargin()
                    )
                }
            }
        }

        val customArgsList = "[\n$customArgs]"
        val clTypeExtensionDef = "{$clTypeExtension}"

        val codeText = """
        |// ************* App Parameters
        |let __timeout: TimeValue | undefined = ${getTimeoutTimeValue()};
        |let __keepAlive: boolean = ${targetConfig.keepalive};
        |let __fast: boolean = ${targetConfig.fastMode};
        |let __federationID: string = 'Unidentified Federation'
        |
        |let __noStart = false; // If set to true, don't start the app.
        |
        |// ************* Custom Command Line Arguments
        |let __additionalCommandLineArgs : __CommandLineOptionSpec = $customArgsList;
        |let __customCommandLineArgs = __CommandLineOptionDefs.concat(__additionalCommandLineArgs);
        |let __customCommandLineUsageDefs = __CommandLineUsageDefs;
        |type __customCLTypeExtension = $clTypeExtensionDef;
        |__customCommandLineUsageDefs[1].optionList = __customCommandLineArgs;
        |const __clUsage = commandLineUsage(__customCommandLineUsageDefs);
        |
        |// Set App parameters using values from the constructor or command line args.
        |// Command line args have precedence over values from the constructor
        |let __processedCLArgs: __ProcessedCommandLineArgs & __customCLTypeExtension;
        |try {
        |    __processedCLArgs =  commandLineArgs(__customCommandLineArgs) as __ProcessedCommandLineArgs & __customCLTypeExtension;
        |} catch (e){
        |    Log.global.error(__clUsage);
        |    throw new Error("Command line argument parsing failed with: " + e);
        |}
        |
        |// Fast Parameter
        |if (__processedCLArgs.fast !== undefined) {
        |    if (__processedCLArgs.fast !== null) {
        |        __fast = __processedCLArgs.fast;
        |    } else {
        |        Log.global.error(__clUsage);
        |        throw new Error("'fast' command line argument is malformed.");
        |    }
        |}
        |
        |// federationID Parameter
        |if (__processedCLArgs.id !== undefined) {
        |    if (__processedCLArgs.id !== null) {
        |        __federationID = __processedCLArgs.id;
        |    } else {
        |        Log.global.error(__clUsage);
        |        throw new Error("'id (federationID)' command line argument is malformed.");
        |    }
        |}
        |
        |// KeepAlive parameter
        |if (__processedCLArgs.keepalive !== undefined) {
        |    if (__processedCLArgs.keepalive !== null) {
        |        __keepAlive = __processedCLArgs.keepalive;
        |    } else {
        |        Log.global.error(__clUsage);
        |        throw new Error("'keepalive' command line argument is malformed.");
        |    }
        |}
        |
        |// Timeout parameter
        |if (__processedCLArgs.timeout !== undefined) {
        |    if (__processedCLArgs.timeout !== null) {
        |        __timeout = __processedCLArgs.timeout;
        |    } else {
        |        Log.global.error(__clUsage);
        |        throw new Error("'timeout' command line argument is malformed.");
        |    }
        |}
        |
        |// Logging parameter (not a constructor parameter, but a command line option)
        |if (__processedCLArgs.logging !== undefined) {
        |    if (__processedCLArgs.logging !== null) {
        |        Log.global.level = __processedCLArgs.logging;
        |    } else {
        |        Log.global.error(__clUsage);
        |        throw new Error("'logging' command line argument is malformed.");
        |    }
        |} else {
        |    Log.global.level = Log.levels.${targetConfig.logLevel.name}; // Default from target property.
        |}
        |
        |// Help parameter (not a constructor parameter, but a command line option)
        |// NOTE: this arg has to be checked after logging, because the help mode should
        |// suppress debug statements from it changes logging
        |if (__processedCLArgs.help === true) {
        |    Log.global.error(__clUsage);
        |    __noStart = true;
        |    // Don't execute the app if the help flag is given.
        |}
        |
        |// Now the logging property has been set to its final value,
        |// log information about how command line arguments were set,
        |// but only if not in help mode.
        |
        |// Runtime command line arguments 
        |if (__processedCLArgs.fast !== undefined && __processedCLArgs.fast !== null
        |    && !__noStart) {
        |    Log.global.info("'fast' property overridden by command line argument.");
        |}
        |if (__processedCLArgs.id !== undefined && __processedCLArgs.id !== null
        |    && !__noStart) {
        |    Log.global.info("'id (federationID)' property overridden by command line argument.");
        |}
        |if (__processedCLArgs.keepalive !== undefined && __processedCLArgs.keepalive !== null
        |    && !__noStart) {
        |    Log.global.info("'keepalive' property overridden by command line argument.");
        |}
        |if (__processedCLArgs.timeout !== undefined && __processedCLArgs.timeout !== null
        |    && !__noStart) {
        |    Log.global.info("'timeout' property overridden by command line argument.");
        |}
        |if (__processedCLArgs.logging !== undefined && __processedCLArgs.logging !== null
        |    && !__noStart) {
        |     Log.global.info("'logging' property overridden by command line argument.");
        |}
        |
        |// Custom command line arguments
        |${logCustomCLArgs(mainParameters)}
        |// Assign custom command line arguments
        |${assignCustomCLArgs(mainParameters)}
        |
    """.trimMargin()

        return Pair(mainParameters, codeText)
    }
}
