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

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.*
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.*
import org.lflang.scoping.LFGlobalScopeProvider
import java.lang.StringBuilder
import java.util.*

/** Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class TsGenerator(
    private val tsFileConfig: TsFileConfig,
    errorReporter: ErrorReporter,
    private val scopeProvider: LFGlobalScopeProvider
) : GeneratorBase(tsFileConfig, errorReporter) {

    companion object {
        /** Path to the Cpp lib directory (relative to class path)  */
        const val LIB_PATH = "/lib/TS"

        /**
         * Names of the configuration files to check for and copy to the generated
         * source package root if they cannot be found in the source directory.
         */
        val CONFIG_FILES = arrayOf("package.json", "tsconfig.json", "babel.config.js")

        /**
         * Files to be copied from the reactor-ts submodule into the generated
         * source directory.
         */
        val RUNTIME_FILES = arrayOf("cli.ts", "command-line-args.d.ts",
            "command-line-usage.d.ts", "component.ts", "federation.ts", "reaction.ts",
            "reactor.ts", "microtime.d.ts", "nanotimer.d.ts", "time.ts", "ulog.d.ts",
            "util.ts")
    }

    init {
        // Set defaults for federate compilation.
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags.add("-O2")
    }

    /**
     * Set of parameters (AST elements) associated with the main reactor.
     */
    var mainParameters = hashSetOf<Parameter>()

    /**
     * Wrappers for pr(), indent(), and unindent() functions.
     */
    fun prw(builder: StringBuilder, text: Any) {
        pr(builder, text)
    }
    fun indentw(builder: StringBuilder) {
        indent(builder)
    }
    fun unindentw(builder: StringBuilder) {
        unindent(builder)
    }
    fun federationRTIPropertiesW() = federationRTIProperties
    fun getTargetValueW(v: Value): String {
        return getTargetValue(v)
    }
    fun getTargetTypeW(p: Parameter): String {
        return getTargetType(p.inferredType)
    }
    fun getTargetTypeW(state: StateVar): String {
        return getTargetType(state)
    }
    fun getTargetTypeW(a: Action): String {
        return getTargetType(a)
    }
    fun getTargetTypeW(p: Port): String {
        return getTargetType(p)
    }
    fun getTargetTypeW(t: Type): String {
        return getTargetType(t)
    }
    fun getInitializerListW(state: StateVar): List<String> {
        return getInitializerList(state)
    }
    fun getInitializerListW(param: Parameter): List<String> {
        return getInitializerList(param)
    }
    fun generateVarRefW(reference: VarRef): String {
        return generateVarRef(reference)
    }

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2,
                            context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (errorsOccurred()) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
            return
        }

        fileConfig.deleteDirectory(fileConfig.srcGenPath)
        for (runtimeFile in RUNTIME_FILES) {
            copyFileFromClassPath(
                "/lib/TS/reactor-ts/src/core/$runtimeFile",
                tsFileConfig.tsCoreGenPath().resolve(runtimeFile).toString())
        }

        /**
         * Check whether configuration files are present in the same directory
         * as the source file. For those that are missing, install a default
         * If the given filename is not present in the same directory as the source
         * file, copy a default version of it from /lib/TS/.
         */
        for (configFile in CONFIG_FILES) {
            val configFileDest = fileConfig.srcGenPath.resolve(configFile).toString()
            val configFileInSrc = fileConfig.srcPath.resolve(configFile).toString()
            if (fsa.isFile(configFileInSrc)) {
                // TODO(hokeun): Check if this logic is still necessary.
                println("Copying '" + configFile + "' from " + fileConfig.srcPath)
                copyFileFromClassPath(configFileInSrc, configFileDest)
            } else {
                println(
                    "No '" + configFile + "' exists in " + fileConfig.srcPath +
                            ". Using default configuration."
                )
                copyFileFromClassPath("/lib/TS/$configFile", configFileDest)
            }
        }

        for (federate in federates) {
            var tsFileName = fileConfig.name
            if (isFederated) {
                tsFileName += '_' + federate.name
            }

            val tsFilePath = tsFileConfig.tsSrcGenPath().resolve(tsFileName + ".ts")

            val tsCode = StringBuilder()

            val preambleGenerator = TsPreambleGenerator(fileConfig.srcFile.toPath())
            tsCode.append(preambleGenerator.generatePreamble())

            val parameterGenerator = TsParameterGenerator(fileConfig, targetConfig, reactors)
            tsCode.append(parameterGenerator.generatePrameters())

            val reactorGenerator = TsReactorGenerator(this, errorReporter)
            for (reactor in reactors) {
                tsCode.append(reactorGenerator.generateReactor(reactor, federate))
            }
            fsa.generateFile(fileConfig.srcGenBasePath.relativize(tsFilePath).toString(),
                tsCode.toString())
        }
    }

    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        if (action.type !== null) {
            return getTargetType(action.type)
        } else {
            return "Present"
        }
    }

    /** Given a representation of time that may possibly include units,
     *  return a string that TypeScript can recognize as a value.
     *  @param value Literal that represents a time value.
     *  @return A string, as "[ timeLiteral, TimeUnit.unit]" .
     */
    override fun timeInTargetLanguage(value: TimeValue): String {
        return if (value.unit != TimeUnit.NONE) {
            """TimeValue.${value.unit}(${value.time})"""
        } else {
            // The value must be zero.
            "TimeValue.zero()"
        }
    }

    override fun generateDelayBody(action: Action, port: VarRef): String {
        return """actions.${action.name}.schedule(0, ${generateVarRef(port)} as ${getActionType(action)});"""
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        return """${generateVarRef(port)} = ${action.name} as ${getActionType(action)};"""
    }

    override fun generateDelayGeneric(): String {
        return """T extends Present"""
    }

    override fun supportsGenerics(): Boolean {
        return true
    }

    override fun getTargetTimeType(): String {
        return "TimeValue"
    }

    override fun getTargetTagType(): String {
        return "TimeValue"
    }

    override fun getTargetTagIntervalType(): String {
        return this.targetUndefinedType
    }

    override fun getTargetUndefinedType(): String {
        return "Present"
    }

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String {
        return """Array(${size})<${baseType}>"""
    }

    override fun getTargetVariableSizeListType(baseType: String): String {
        return """Array<${baseType}>"""
    }

    override fun getTarget(): Target {
        return Target.TS
    }

}
