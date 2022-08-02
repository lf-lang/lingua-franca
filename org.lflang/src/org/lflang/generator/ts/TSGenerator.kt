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
import org.lflang.ASTUtils
import org.lflang.ErrorReporter
import org.lflang.Target
import org.lflang.TimeValue
import org.lflang.generator.CodeMap
import org.lflang.generator.ExpressionGenerator
import org.lflang.generator.GeneratorBase
import org.lflang.generator.GeneratorResult
import org.lflang.generator.GeneratorUtils
import org.lflang.generator.GeneratorUtils.canGenerate
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.ReactorInstance
import org.lflang.generator.SubContext
import org.lflang.generator.TargetTypes
import org.lflang.graph.InstantiationGraph
import org.lflang.inferredType
import org.lflang.lf.Action
import org.lflang.lf.Expression
import org.lflang.lf.Instantiation
import org.lflang.lf.Parameter
import org.lflang.lf.StateVar
import org.lflang.lf.Type
import org.lflang.lf.VarRef
import org.lflang.model
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.StandardCopyOption
import java.util.*
import kotlin.collections.HashMap

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

        private val VG =
            ExpressionGenerator(::timeInTargetLanguage) { param -> "this.${param.name}.get()" }

        fun timeInTargetLanguage(value: TimeValue): String {
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

    fun getTargetValueW(expr: Expression): String = VG.getTargetValue(expr, false)
    fun getTargetTypeW(p: Parameter): String = TSTypes.getTargetType(p.inferredType)
    fun getTargetTypeW(state: StateVar): String = TSTypes.getTargetType(state)
    fun getTargetTypeW(t: Type): String = TSTypes.getTargetType(t)

    fun getInitializerListW(state: StateVar): List<String> = VG.getInitializerList(state)
    fun getInitializerListW(param: Parameter): List<String> = VG.getInitializerList(param)
    fun getInitializerListW(param: Parameter, i: Instantiation): List<String> =
        VG.getInitializerList(param, i)

    fun getInstantiationGraph(): InstantiationGraph? {
        return this.instantiationGraph;
    }

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param context The context of this build.
     */
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)

        instantiationGraph

        if (!canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return
        if (!isOsCompatible()) return

        // createMainReactorInstance()

        clean(context)
        copyRuntime()
        copyConfigFiles()

        val codeMaps = HashMap<Path, CodeMap>()
        val dockerGenerator = TSDockerGenerator(false)
        generateCode(codeMaps, dockerGenerator, TSFederateConfig.createFederateConfig(resource.model.preambles))
        if (targetConfig.dockerOptions != null) {
            dockerGenerator.writeDockerFiles(tsFileConfig.tsDockerComposeFilePath())
        }
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
     * If a main or federated reactor has been declared, create a ReactorInstance of it.
     * This will assign levels to reactions; then, if the program is federated,
     * an AST transformation is performed to disconnect connections between federates.
     */
    private fun createMainReactorInstance() {
        if (mainDef != null) {
            if (main == null) {
                // Recursively build instances. This is done once because
                // it is the same for all federates.
                main = ReactorInstance(
                    ASTUtils.toDefinition(mainDef.reactorClass), errorReporter,
                    unorderedReactions
                )
                val reactionInstanceGraph = main.assignLevels()
                if (reactionInstanceGraph.nodeCount() > 0) {
                    errorReporter.reportError("Main reactor has causality cycles. Skipping code generation.")
                    return
                }
                // Inform the run-time of the breadth/parallelism of the reaction graph
                val breadth = reactionInstanceGraph.breadth
                if (breadth == 0) {
                    errorReporter.reportWarning("The program has no reactions")
                } else {
                    targetConfig.compileDefinitions["LF_REACTION_GRAPH_BREADTH"] = reactionInstanceGraph.breadth.toString()
                }
            }

            // Force reconstruction of dependence information.
//            if (isFederated) {
//                // FIXME: The following operation must be done after levels are assigned.
//                //  Removing these ports before that will cause incorrect levels to be assigned.
//                //  See https://github.com/lf-lang/lingua-franca/discussions/608
//                //  For now, avoid compile errors by removing disconnected network ports before
//                //  assigning levels.
//                removeRemoteFederateConnectionPorts(main)
//                // There will be AST transformations that invalidate some info
//                // cached in ReactorInstance.
//                main.clearCaches(false)
//            } FIXME: @hokeun
        }
    }

    /**
     * Generate the code corresponding to [federate], recording any resulting mappings in [codeMaps].
     */
    private fun generateCode(
        codeMaps: MutableMap<Path, CodeMap>,
        dockerGenerator: TSDockerGenerator,
        federateConfig: TSFederateConfig?
    ) {
        var tsFileName = fileConfig.name

        val tsFilePath = tsFileConfig.tsSrcGenPath().resolve("$tsFileName.ts")

        val tsCode = StringBuilder()

        val preambleGenerator = TSImportPreambleGenerator(fileConfig.srcFile,
            targetConfig.protoFiles)
        tsCode.append(preambleGenerator.generatePreamble())

        val parameterGenerator = TSParameterPreambleGenerator(this, fileConfig, targetConfig, reactors)
        val (mainParameters, parameterCode) = parameterGenerator.generateParameters()
        tsCode.append(parameterCode)

        val reactorGenerator = TSReactorGenerator(this, errorReporter, targetConfig)
        for (reactor in reactors) {
            tsCode.append(reactorGenerator.generateReactorClasses(reactor, federateConfig))
        }

        tsCode.append(reactorGenerator.generateMainReactorInstanceAndStart(federateConfig, this.mainDef, mainParameters))

        val codeMap = CodeMap.fromGeneratedCode(tsCode.toString())
        codeMaps[tsFilePath] = codeMap
        FileUtil.writeToFile(codeMap.generatedCode, tsFilePath)

        if (targetConfig.dockerOptions != null) {
            dockerGenerator.addFile(dockerGenerator.fromData(tsFileName, tsFileConfig))
        }
    }

    private fun compile(resource: Resource, parsingContext: LFGeneratorContext) {

        GeneratorUtils.refreshProject(resource, parsingContext.mode)

        if (parsingContext.cancelIndicator.isCanceled) return
        parsingContext.reportProgress("Transpiling to JavaScript...", 70)
        transpile(parsingContext.cancelIndicator)

        if (parsingContext.cancelIndicator.isCanceled) return
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
     * Inform the context of the results of a compilation.
     * @param context The context of the compilation.
     */
    private fun concludeCompilation(context: LFGeneratorContext, codeMaps: Map<Path, CodeMap>) {
        if (errorReporter.errorsOccurred) {
            context.unsuccessfulFinish()
        } else {
            context.finish(
                GeneratorResult.Status.COMPILED, fileConfig.name + ".js",
                fileConfig.srcGenPkgPath.resolve("dist"), fileConfig, codeMaps, "node"
            )
        }
    }

    private fun isOsCompatible(): Boolean {
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
