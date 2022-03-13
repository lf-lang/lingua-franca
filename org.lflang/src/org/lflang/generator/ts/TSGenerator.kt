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
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.ErrorReporter
import org.lflang.InferredType
import org.lflang.ASTUtils
import org.lflang.Target
import org.lflang.TimeValue
import org.lflang.federated.FederateInstance
import org.lflang.federated.launcher.FedTSLauncher
import org.lflang.federated.serialization.SupportedSerializers
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorBase
import org.lflang.generator.GeneratorResult
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.GeneratorUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.PrependOperator
import org.lflang.generator.SubContext
import org.lflang.generator.TargetTypes
import org.lflang.generator.ValueGenerator
import org.lflang.generator.GeneratorUtils.canGenerate
import org.lflang.inferredType
import org.lflang.lf.Action
import org.lflang.lf.Delay
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.StateVar
import org.lflang.lf.Type
import org.lflang.lf.Value
import org.lflang.lf.VarRef
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.StandardCopyOption
import java.util.*

private const val NO_NPM_MESSAGE = "The TypeScript target requires npm >= 6.14.4. " +
        "For installation instructions, see: https://www.npmjs.com/get-npm. \n" +
        "Auto-compiling can be disabled using the \"no-compile: true\" target property."

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
        const val LIB_PATH = "/lib/ts"

        /**
         * Names of the configuration files to check for and copy to the generated
         * source package root if they cannot be found in the source directory.
         */
        val CONFIG_FILES = arrayOf("package.json", "tsconfig.json", "babel.config.js", ".eslintrc.json")

        /**
         * Files to be copied from the reactor-ts submodule into the generated
         * source directory.
         */
        val RUNTIME_FILES = arrayOf("action.ts", "bank.ts", "cli.ts", "command-line-args.d.ts",
            "command-line-usage.d.ts", "component.ts", "event.ts", "federation.ts", "internal.ts",
            "reaction.ts", "reactor.ts", "microtime.d.ts", "multiport.ts", "nanotimer.d.ts", "port.ts",
            "state.ts", "strings.ts", "time.ts", "trigger.ts", "types.ts", "ulog.d.ts", "util.ts")

        private val VG = ValueGenerator(::timeInTargetLanguage) { param -> "this.${param.name}.get()" }

        private fun timeInTargetLanguage(value: TimeValue): String {
            return if (value.unit != null) {
                "TimeValue.${value.unit.canonicalName}(${value.magnitude})"
            } else {
                // The value must be zero.
                "TimeValue.zero()"
            }
        }

        /**
         * The percent progress associated with having collected all JS/TS dependencies.
         */
        private const val COLLECTED_DEPENDENCIES_PERCENT_PROGRESS
            = (IntegratedBuilder.GENERATED_PERCENT_PROGRESS + IntegratedBuilder.COMPILED_PERCENT_PROGRESS) / 2
    }

    init {
        // Set defaults for federate compilation.
        targetConfig.compiler = "gcc"
        targetConfig.compilerFlags.add("-O2")
    }

    // Wrappers to expose GeneratorBase methods.
    fun federationRTIPropertiesW() = federationRTIProperties

    fun getTargetValueW(v: Value): String = VG.getTargetValue(v, false)
    fun getTargetTypeW(p: Parameter): String = TSTypes.getTargetType(p.inferredType)
    fun getTargetTypeW(state: StateVar): String = TSTypes.getTargetType(state)
    fun getTargetTypeW(t: Type): String = TSTypes.getTargetType(t)

    fun getInitializerListW(state: StateVar): List<String> = VG.getInitializerList(state)
    fun getInitializerListW(param: Parameter): List<String> = VG.getInitializerList(param)
    fun getInitializerListW(param: Parameter, i: Instantiation): List<String> =
        VG.getInitializerList(param, i)

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param context The context of this build.
     */
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)

        if (!canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return
        if (!isOsCompatible()) return
        
        // FIXME: The following operation must be done after levels are assigned.
        //  Removing these ports before that will cause incorrect levels to be assigned.
        //  See https://github.com/lf-lang/lingua-franca/discussions/608
        //  For now, avoid compile errors by removing disconnected network ports before
        //  assigning levels.
        removeRemoteFederateConnectionPorts(null);
        
        clean(context)
        copyRuntime()
        copyConfigFiles()

        val codeMaps = HashMap<Path, CodeMap>()
        for (federate in federates) generateCode(federate, codeMaps)
        // For small programs, everything up until this point is virtually instantaneous. This is the point where cancellation,
        // progress reporting, and IDE responsiveness become real considerations.
        if (targetConfig.noCompile) {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(null))
        } else {
            context.reportProgress(
                "Code generation complete. Collecting dependencies...", IntegratedBuilder.GENERATED_PERCENT_PROGRESS
            )
            if (shouldCollectDependencies(context)) collectDependencies(resource, context)
            if (errorsOccurred()) {
                context.unsuccessfulFinish();
                return;
            }
            if (targetConfig.protoFiles.size != 0) {
                protoc()
            } else {
                println("No .proto files have been imported. Skipping protocol buffer compilation.")
            }
            val parsingContext = SubContext(context, COLLECTED_DEPENDENCIES_PERCENT_PROGRESS, 100)
            if (
                !context.cancelIndicator.isCanceled
                && passesChecks(TSValidator(tsFileConfig, errorReporter, codeMaps), parsingContext)
            ) {
                if (context.mode == LFGeneratorContext.Mode.LSP_MEDIUM) {
                    context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(codeMaps))
                } else {
                    compile(resource, parsingContext)
                    concludeCompilation(context, codeMaps)
                }
            } else {
                context.unsuccessfulFinish()
            }
        }
    }

    /**
     * Clean up the src-gen directory as needed to prepare for code generation.
     */
    private fun clean(context: LFGeneratorContext) {
        // Dirty shortcut for integrated mode: Delete nothing, saving the node_modules directory to avoid re-running pnpm.
        if (context.mode != LFGeneratorContext.Mode.LSP_MEDIUM) FileUtil.deleteDirectory(
            fileConfig.srcGenPath
        )
    }

    /**
     * Copy the TypeScript runtime so that it is accessible to the generated code.
     */
    private fun copyRuntime() {
        for (runtimeFile in RUNTIME_FILES) {
            FileUtil.copyFileFromClassPath(
                "$LIB_PATH/reactor-ts/src/core/$runtimeFile",
                tsFileConfig.tsCoreGenPath().resolve(runtimeFile)
            )
        }
    }

    /**
     * For each configuration file that is not present in the same directory
     * as the source file, copy a default version from $LIB_PATH/.
     */
    private fun copyConfigFiles() {
        for (configFile in CONFIG_FILES) {
            val configFileDest = fileConfig.srcGenPath.resolve(configFile)
            val configFileInSrc = fileConfig.srcPath.resolve(configFile)
            if (configFileInSrc.toFile().exists()) {
                println("Copying '" + configFile + "' from " + fileConfig.srcPath)
                Files.copy(configFileInSrc, configFileDest, StandardCopyOption.REPLACE_EXISTING)
            } else {
                println(
                    "No '" + configFile + "' exists in " + fileConfig.srcPath +
                            ". Using default configuration."
                )
                FileUtil.copyFileFromClassPath("$LIB_PATH/$configFile", configFileDest)
            }
        }
    }

    /**
     * Generate the code corresponding to [federate], recording any resulting mappings in [codeMaps].
     */
    private fun generateCode(federate: FederateInstance, codeMaps: MutableMap<Path, CodeMap>) {
        var tsFileName = fileConfig.name
        // TODO(hokeun): Consider using FedFileConfig when enabling federated execution for TypeScript.
        // For details, see https://github.com/icyphy/lingua-franca/pull/431#discussion_r676302102
        if (isFederated) {
            tsFileName += '_' + federate.name
        }

        val tsFilePath = tsFileConfig.tsSrcGenPath().resolve("$tsFileName.ts")

        val tsCode = StringBuilder()

        val preambleGenerator = TSImportPreambleGenerator(fileConfig.srcFile,
            targetConfig.protoFiles)
        tsCode.append(preambleGenerator.generatePreamble())

        val parameterGenerator = TSParameterPreambleGenerator(this, fileConfig, targetConfig, reactors)
        val (mainParameters, parameterCode) = parameterGenerator.generateParameters()
        tsCode.append(parameterCode)

        val reactorGenerator = TSReactorGenerator(this, errorReporter)
        for (reactor in reactors) {
            tsCode.append(reactorGenerator.generateReactor(reactor, federate))
        }
        tsCode.append(reactorGenerator.generateReactorInstanceAndStart(this.mainDef, mainParameters))
        val codeMap = CodeMap.fromGeneratedCode(tsCode.toString())
        codeMaps[tsFilePath] = codeMap
        FileUtil.writeToFile(codeMap.generatedCode, tsFilePath)

        if (targetConfig.dockerOptions != null && isFederated) {
            println("WARNING: Federated Docker file generation is not supported on the Typescript target. No docker file is generated.")
        } else if (targetConfig.dockerOptions != null) {
            val dockerFilePath = fileConfig.srcGenPath.resolve("$tsFileName.Dockerfile")
            val dockerComposeFile = fileConfig.srcGenPath.resolve("docker-compose.yml")
            val dockerGenerator = TSDockerGenerator(tsFileName)
            println("docker file written to $dockerFilePath")
            FileUtil.writeToFile(dockerGenerator.generateDockerFileContent(), dockerFilePath)
            FileUtil.writeToFile(
                dockerGenerator.generateDockerComposeFileContent(),
                dockerComposeFile
            )
        }
    }

    private fun compile(resource: Resource, parsingContext: LFGeneratorContext) {

        GeneratorUtils.refreshProject(resource, parsingContext.mode)

        if (parsingContext.cancelIndicator.isCanceled) return
        parsingContext.reportProgress("Transpiling to JavaScript...", 70)
        transpile(parsingContext.cancelIndicator)

        if (parsingContext.cancelIndicator.isCanceled) return
        if (isFederated) {
            parsingContext.reportProgress("Generating federation infrastructure...", 90)
            generateFederationInfrastructure()
        }
    }

    /**
     * Return whether it is advisable to install dependencies.
     */
    private fun shouldCollectDependencies(context: LFGeneratorContext): Boolean =
        context.mode != LFGeneratorContext.Mode.LSP_MEDIUM || !fileConfig.srcGenPkgPath.resolve("node_modules").toFile().exists()

    /**
     * Collect the dependencies in package.json and their
     * transitive dependencies.
     * @param resource The Lingua Franca source file at
     * which to report any errors
     * @param context The context of this build.
     */
    private fun collectDependencies(resource: Resource, context: LFGeneratorContext) {

        Files.createDirectories(fileConfig.srcGenPkgPath) // may throw

        val pnpmInstall = commandFactory.createCommand(
            "pnpm",
            listOf("install"),
            fileConfig.srcGenPkgPath,
            false // only produce a warning if command is not found
        )

        // Attempt to use pnpm, but fall back on npm if it is not available.
        if (pnpmInstall != null) {
            val ret = pnpmInstall.run(context.cancelIndicator)
            if (ret != 0) {
                val errors: String = pnpmInstall.errors.toString()
                errorReporter.reportError(
                    GeneratorUtils.findTarget(resource),
                    "ERROR: pnpm install command failed" + if (errors.isBlank()) "." else ":\n$errors")
            }
        } else {
            errorReporter.reportWarning(
                "Falling back on npm. To prevent an accumulation of replicated dependencies, " +
                        "it is highly recommended to install pnpm globally (npm install -g pnpm).")
            val npmInstall = commandFactory.createCommand("npm", listOf("install"), fileConfig.srcGenPkgPath)

            if (npmInstall == null) {
                errorReporter.reportError(NO_NPM_MESSAGE)
                return
            }

            if (npmInstall.run(context.cancelIndicator) != 0) {
                errorReporter.reportError(
                    GeneratorUtils.findTarget(resource),
                    "ERROR: npm install command failed: " + npmInstall.errors.toString())
                errorReporter.reportError(
                    GeneratorUtils.findTarget(resource), "ERROR: npm install command failed." +
                        "\nFor installation instructions, see: https://www.npmjs.com/get-npm")
                return
            }
        }
    }

    /**
     * Invoke the protocol buffers compiler on all .proto
     * files in the project directory.
     */
    private fun protoc() {
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
            errorReporter.reportError("Processing .proto files requires libprotoc >= 3.6.1.")
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
            errorReporter.reportError("protoc failed with error code $returnCode.")
        }
        // FIXME: report errors from this command.
    }

    /**
     * Run checks on the generated TypeScript.
     * @return whether the checks pass.
     */
    private fun passesChecks(validator: TSValidator, parsingContext: LFGeneratorContext): Boolean {
        parsingContext.reportProgress("Linting generated code...", 0)
        validator.doLint(parsingContext)
        if (errorsOccurred()) return false
        parsingContext.reportProgress("Validating generated code...", 25)
        validator.doValidate(parsingContext)
        return !errorsOccurred()
    }

    /**
     * Transpile TypeScript to JavaScript.
     */
    private fun transpile(cancelIndicator: CancelIndicator) {
        println("Compiling")
        val babel = commandFactory.createCommand("npm", listOf("run", "build"), fileConfig.srcGenPkgPath)

        if (babel == null) {
            errorReporter.reportError(NO_NPM_MESSAGE)
            return
        }

        if (babel.run(cancelIndicator) == 0) {
            println("SUCCESS (compiling generated TypeScript code)")
        } else {
            errorReporter.reportError("Compiler failed with the following errors:\n${babel.errors}")
        }
    }

    /**
     * Set up the runtime infrastructure and federation
     * launcher script.
     */
    private fun generateFederationInfrastructure() {
        // Create bin directory for the script.
        if (!Files.exists(fileConfig.binPath)) {
            Files.createDirectories(fileConfig.binPath)
        }
        // Generate script for launching federation
        val launcher = FedTSLauncher(targetConfig, fileConfig, errorReporter)
        launcher.createLauncher(federates, federationRTIPropertiesW())
        // TODO(hokeun): Modify this to make this work with standalone RTI.
        // If this is a federated execution, generate C code for the RTI.
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
    }

    /**
     * Inform the context of the results of a compilation.
     * @param context The context of the compilation.
     */
    private fun concludeCompilation(context: LFGeneratorContext, codeMaps: Map<Path, CodeMap>) {
        if (errorReporter.errorsOccurred) {
            context.unsuccessfulFinish()
        } else {
            if (isFederated) {
                context.finish(GeneratorResult.Status.COMPILED, fileConfig.name, fileConfig, codeMaps)
            } else {
                context.finish(
                    GeneratorResult.Status.COMPILED, fileConfig.name + ".js",
                    fileConfig.srcGenPkgPath.resolve("dist"), fileConfig, codeMaps, "node"
                )
            }
        }
    }

    private fun isOsCompatible(): Boolean {
        if (isFederated && GeneratorUtils.isHostWindows()) {
            errorReporter.reportError(
                "Federated LF programs with a TypeScript target are currently not supported on Windows. Exiting code generation."
            )
            return false
        }
        return true
    }

    override fun getTargetTypes(): TargetTypes = TSTypes

    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        return if (action.type != null) {
            TSTypes.getTargetType(action.type)
        } else {
            "Present"
        }
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
     * @param serializer The serializer used on the connection.
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
        isPhysical: Boolean,
        serializer: SupportedSerializers
    ): String {
        return with(PrependOperator) {"""
        |// FIXME: For now assume the data is a Buffer, but this is not checked.
        |// Replace with ProtoBufs or MessagePack.
        |// generateNetworkReceiverBody
        |if (${action.name} !== undefined) {
        |    ${receivingPort.container.name}.${receivingPort.variable.name} = ${action.name}.toString(); // defaults to utf8 encoding
        |}
        """.trimMargin()}
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network. This base class throws an exception.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @throws UnsupportedOperationException If the target does not support this operation.
     * @param serializer The serializer used on the connection.
     */
    override fun generateNetworkSenderBody(
        sendingPort: VarRef,
        receivingPort: VarRef,
        receivingPortID: Int,
        sendingFed: FederateInstance,
        sendingBankIndex: Int,
        sendingChannelIndex: Int,
        receivingFed: FederateInstance,
        type: InferredType,
        isPhysical: Boolean,
        delay: Delay?,
        serializer: SupportedSerializers
    ): String {
        return with(PrependOperator) {"""
        |// FIXME: For now assume the data is a Buffer, but this is not checked.
        |// Use SupportedSerializers for determining the serialization later.
        |if (${sendingPort.container.name}.${sendingPort.variable.name} !== undefined) {
        |    let buf = Buffer.from(${sendingPort.container.name}.${sendingPort.variable.name})
        |    this.util.sendRTITimedMessage(buf, ${receivingFed.id}, ${receivingPortID});
        |}
        """.trimMargin()}
    }


    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     *
     * @param port The port to generate the control reaction for
     * @param portID The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel if a multiport
     * @param delay The delay value imposed on the connection using after
     */
    override fun generateNetworkOutputControlReactionBody(
        port: VarRef?,
        portID: Int,
        receivingFederateID: Int,
        sendingBankIndex: Int,
        sendingChannelIndex: Int,
        delay: Delay?
    ): String? {
        return with(PrependOperator) {"""
        |// TODO(hokeun): Figure out what to do for generateNetworkOutputControlReactionBody
        """.trimMargin()}
    }

    /**
     * Generate code for the body of a reaction that waits long enough so that the status
     * of the trigger for the given port becomes known for the current logical time.
     *
     * @param port The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     * that have port as their trigger or source
     */
    override fun generateNetworkInputControlReactionBody(receivingPortID: Int, maxSTP: TimeValue?): String? {
        return with(PrependOperator) {"""
        |// TODO(hokeun): Figure out what to do for generateNetworkInputControlReactionBody
        """.trimMargin()}
    }

    /**
     * Add necessary code to the source and necessary build supports to
     * enable the requested serializations in 'enabledSerializations'
     */
    override fun enableSupportForSerializationIfApplicable(cancelIndicator: CancelIndicator?) {
        for (serializer in enabledSerializers) {
            when (serializer) {
                SupportedSerializers.NATIVE -> {
                    // No need to do anything at this point.
                    println("Native serializer is enabled.")
                }
                else -> throw UnsupportedOperationException("Unsupported serializer: $serializer");
            }
        }
    }

    // Virtual methods.
    override fun generateDelayBody(action: Action, port: VarRef): String {
        return "actions.${action.name}.schedule(0, ${ASTUtils.generateVarRef(port)} as ${getActionType(action)});"
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        return "${ASTUtils.generateVarRef(port)} = ${action.name} as ${getActionType(action)};"
    }

    override fun generateDelayGeneric(): String {
        return "T extends Present"
    }

    override fun getTarget(): Target {
        return Target.TS
    }

    override fun generateAfterDelaysWithVariableWidth() = false
}
