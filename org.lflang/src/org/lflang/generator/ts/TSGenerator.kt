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
import org.lflang.ASTUtils.isInitialized
import org.lflang.Target
import org.lflang.generator.FederateInstance
import org.lflang.generator.GeneratorBase
import org.lflang.generator.PrependOperator
import org.lflang.lf.*
import org.lflang.scoping.LFGlobalScopeProvider
import java.lang.StringBuilder
import java.nio.file.Files
import java.util.LinkedList

/**
 * Generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class TSGenerator(
    private val tsFileConfig: TSFileConfig,
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

    // Wrappers to expose GeneratorBase methods.
    fun prw(builder: StringBuilder, text: Any) = pr(builder, text)
    fun indentw(builder: StringBuilder) = indent(builder)
    fun unindentw(builder: StringBuilder) = unindent(builder)

    fun federationRTIPropertiesW() = federationRTIProperties

    fun getTargetValueW(v: Value): String = getTargetValue(v)
    fun getTargetTypeW(p: Parameter): String = getTargetType(p.inferredType)
    fun getTargetTypeW(state: StateVar): String = getTargetType(state)
    fun getTargetTypeW(a: Action): String = getTargetType(a)
    fun getTargetTypeW(p: Port): String = getTargetType(p)
    fun getTargetTypeW(t: Type): String = getTargetType(t)

    fun getInitializerListW(state: StateVar): List<String> = getInitializerList(state)
    fun getInitializerListW(param: Parameter): List<String> = getInitializerList(param)
    fun getInitializerListW(param: Parameter, i: Instantiation): List<String> =
        getInitializerList(param, i)
    fun generateVarRefW(reference: VarRef): String =generateVarRef(reference)

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: Undocumented argument. No idea what this is.
     */
    // TODO(hokeun): Split this method into smaller methods.
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
            // TODO(hokeun): Consider using FedFileConfig when enabling federated execution for TypeScript.
            // For details, see https://github.com/icyphy/lingua-franca/pull/431#discussion_r676302102
            if (isFederated) {
                tsFileName += '_' + federate.name
            }

            val tsFilePath = tsFileConfig.tsSrcGenPath().resolve("$tsFileName.ts")

            val tsCode = StringBuilder()

            val preambleGenerator = TSPreambleGenerator(fileConfig.srcFile,
                targetConfig.protoFiles)
            tsCode.append(preambleGenerator.generatePreamble())

            val parameterGenerator = TSParameterGenerator(this, fileConfig, targetConfig, reactors)
            val (mainParameters, parameterCode) = parameterGenerator.generateParameters()
            tsCode.append(parameterCode)

            val reactorGenerator = TSReactorGenerator(this, errorReporter)
            for (reactor in reactors) {
                reactorGenerator.generateReactor(reactor, federate)
            }
            reactorGenerator.generateReactorInstanceAndStart(this.mainDef, mainParameters)
            tsCode.append(reactorGenerator.getCode())
            fsa.generateFile(fileConfig.srcGenBasePath.relativize(tsFilePath).toString(),
                tsCode.toString())
        }

        // Run necessary commands.

        Files.createDirectories(fileConfig.srcGenPkgPath) // may throw

        val pnpmInstall = commandFactory.createCommand(
            "pnpm",
            listOf("install"),
            fileConfig.srcGenPkgPath,
            false // only produce a warning if command is not found
        )

        // Attempt to use pnpm, but fall back on npm if it is not available.
        if (pnpmInstall != null) {
            val ret = pnpmInstall.run()
            if (ret != 0) {
                errorReporter.reportError(findTarget(resource),
                    "ERROR: pnpm install command failed: " + pnpmInstall.errors.toString())
            }
        } else {
            errorReporter.reportWarning(
                "Falling back on npm. To prevent an accumulation of replicated dependencies, " +
                        "it is highly recommended to install pnpm globally (npm install -g pnpm).")
            val npmInstall = commandFactory.createCommand("npm", listOf("install"), fileConfig.srcGenPkgPath)

            if (npmInstall == null) {
                errorReporter.reportError(
                    "The TypeScript target requires npm >= 6.14.4. " +
                            "For installation instructions, see: https://www.npmjs.com/get-npm. \n" +
                            "Auto-compiling can be disabled using the \"no-compile: true\" target property.")
                return
            }

            if (npmInstall.run() != 0) {
                errorReporter.reportError(findTarget(resource),
                    "ERROR: npm install command failed: " + npmInstall.errors.toString())
                errorReporter.reportError(findTarget(resource), "ERROR: npm install command failed." +
                        "\nFor installation instructions, see: https://www.npmjs.com/get-npm")
                return
            }
        }

        refreshProject()

        // Invoke the protocol buffers compiler on all .proto files in the project directory
        // Assumes protoc compiler has been installed on this machine

        // First test if the project directory contains any .proto files
        if (targetConfig.protoFiles.size != 0) {
            // For more info, see: https://www.npmjs.com/package/ts-protoc-gen

            // FIXME: Check whether protoc is installed and provides hints how to install if it cannot be found.
            val protocArgs = LinkedList<String>()
            val tsOutPath = tsFileConfig.srcPath.relativize(tsFileConfig.tsSrcGenPath())

            protocArgs.addAll(
                listOf(
                "--plugin=protoc-gen-ts=" + tsFileConfig.srcGenPkgPath.resolve("node_modules").resolve(".bin").resolve("protoc-gen-ts"),
                    "--js_out=import_style=commonjs,binary:$tsOutPath",
                    "--ts_out=$tsOutPath"
                )
            )
            protocArgs.addAll(targetConfig.protoFiles)
            val protoc = commandFactory.createCommand("protoc", protocArgs, tsFileConfig.srcPath)

            if (protoc == null) {
                errorReporter.reportError("Processing .proto files requires libprotoc >= 3.6.1")
                return
            }

            val returnCode = protoc.run()
            if (returnCode == 0) {
                  // FIXME: this code makes no sense. It is removing 6 chars from a file with a 3-char extension
//                val nameSansProto = fileConfig.name.substring(0, fileConfig.name.length - 6)
//
//                targetConfig.compileAdditionalSources.add(
//                this.fileConfig.getSrcGenPath.resolve(nameSansProto + ".pb-c.c").toString)
//
//                targetConfig.compileLibraries.add('-l')
//                targetConfig.compileLibraries.add('protobuf-c')
            } else {
                errorReporter.reportError("protoc returns error code $returnCode")
            }
            // FIXME: report errors from this command.
        } else {
            println("No .proto files have been imported. Skipping protocol buffer compilation.")
        }

        val errorMessage = "The TypeScript target requires npm >= 6.14.1 to compile the generated code. " +
                "Auto-compiling can be disabled using the \"no-compile: true\" target property."

        // Invoke the compiler on the generated code.
        println("Type Checking")
        val tsc = commandFactory.createCommand("npm", listOf("run", "check-types"), fileConfig.srcGenPkgPath)
        if (tsc == null) {
            errorReporter.reportError(errorMessage);
            return
        }

        if (tsc.run() == 0) {
            // Babel will compile TypeScript to JS even if there are type errors
            // so only run compilation if tsc found no problems.
            //val babelPath = codeGenConfig.outPath + File.separator + "node_modules" + File.separator + ".bin" + File.separator + "babel"
            // Working command  $./node_modules/.bin/babel src-gen --out-dir js --extensions '.ts,.tsx'
            println("Compiling")
            val babel = commandFactory.createCommand("npm", listOf("run", "build"), fileConfig.srcGenPkgPath)

            if (babel == null) {
                errorReporter.reportError(errorMessage);
                return
            }

            if (babel.run() == 0) {
                println("SUCCESS (compiling generated TypeScript code)")
            } else {
                errorReporter.reportError("Compiler failed.")
            }
        } else {
            errorReporter.reportError("Type checking failed.")
        }

        // TODO(hokeun): Modify this to make this work with standalone RTI.
        // If this is a federated execution, generate C code for the RTI.
//        if (isFederated) {
//
//            // Copy the required library files into the target file system.
//            // This will overwrite previous versions.
//            var files = ArrayList("rti.c", "rti.h", "federate.c", "reactor_threaded.c", "reactor.c", "reactor_common.c", "reactor.h", "pqueue.c", "pqueue.h", "util.h", "util.c")
//
//            for (file : files) {
//                copyFileFromClassPath(
//                    File.separator + "lib" + File.separator + "core" + File.separator + file,
//                    fileConfig.getSrcGenPath.toString + File.separator + file
//                )
//            }
//        }
    }

    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        return if (action.type != null) {
            getTargetType(action.type)
        } else {
            "Present"
        }
    }

    /** Given a representation of time that may possibly include units,
     *  return a string that TypeScript can recognize as a value.
     *  @param value Literal that represents a time value.
     *  @return A string, as "[ timeLiteral, TimeUnit.unit]" .
     */
    override fun timeInTargetLanguage(value: TimeValue): String {
        return if (value.unit != TimeUnit.NONE) {
            "TimeValue.${value.unit}(${value.time})"
        } else {
            // The value must be zero.
            "TimeValue.zero()"
        }
    }

    override fun getTargetType(s: StateVar): String {
        val type = super.getTargetType(s)
        return if (!isInitialized(s)) {
            "$type | undefined"
        } else {
            type
        }
    }

    override fun getTargetReference(param: Parameter): String {
        return "this.${param.name}.get()"
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @param isPhysical Indicates whether or not the connection is physical
     */
    override fun generateNetworkReceiverBody(
        action: Action,
        sendingPort: VarRef,
        receivingPort: VarRef,
        receivingPortID: Int,
        sendingFed: FederateInstance,
        receivingFed: FederateInstance,
        receivingBankIndex: Int,
        receivingChannelIndex: Int,
        type: InferredType,
        isPhysical: Boolean
    ): String {
        return with(PrependOperator) {"""
        // FIXME: For now assume the data is a Buffer, but this is not checked.
        // Replace with ProtoBufs or MessagePack.
        |if (${action.name} !== undefined) {
        |    ${receivingPort.container.name}.${receivingPort.variable.name} = 
        |    ${action.name}; // defaults to utf8 encoding
        |}
        """.trimMargin()}
    }

    // Virtual methods.
    override fun generateDelayBody(action: Action, port: VarRef): String {
        return "actions.${action.name}.schedule(0, ${generateVarRef(port)} as ${getActionType(action)});"
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        return "${generateVarRef(port)} = ${action.name} as ${getActionType(action)};"
    }

    override fun generateDelayGeneric(): String {
        return "T extends Present"
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
        return "Array(${size})<${baseType}>"
    }

    override fun getTargetVariableSizeListType(baseType: String): String {
        return "Array<${baseType}>"
    }

    override fun getTarget(): Target {
        return Target.TS
    }
}
