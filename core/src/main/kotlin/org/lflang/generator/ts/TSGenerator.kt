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

import com.google.common.base.Strings
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.util.CancelIndicator
import org.lflang.TimeValue
import org.lflang.ast.DelayedConnectionTransformation
import org.lflang.generator.*
import org.lflang.generator.GeneratorUtils.canGenerate
import org.lflang.generator.docker.CDockerGenerator
import org.lflang.generator.docker.DockerComposeGenerator
import org.lflang.generator.docker.DockerGenerator
import org.lflang.generator.docker.TSDockerGenerator
import org.lflang.lf.Preamble
import org.lflang.model
import org.lflang.target.Target
import org.lflang.target.property.DockerProperty
import org.lflang.target.property.NoCompileProperty
import org.lflang.target.property.ProtobufsProperty
import org.lflang.target.property.RuntimeVersionProperty
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path
import java.util.*

private const val NO_NPM_MESSAGE = "The TypeScript target requires npm >= 6.14.4. " +
        "For installation instructions, see: https://www.npmjs.com/get-npm. \n" +
        "Auto-compiling can be disabled using the \"no-compile: true\" target property."

/**
 * Generator for TypeScript target.
 *
 *  @author Matt Weber
 *  @author Edward A. Lee
 *  @author Marten Lohstroh
 *  @author Christian Menard
 *  @author Hokeun Kim
 */
class TSGenerator(
    context: LFGeneratorContext
) : GeneratorBase(context) {


    val fileConfig: TSFileConfig = context.fileConfig as TSFileConfig
    private var devMode = false

    companion object {

        /** Path to the TS lib directory (relative to class path)  */
        const val LIB_PATH = "/lib/ts"

        /**
         * Names of the configuration files to check for and copy to the generated
         * source package root if they cannot be found in the source directory.
         */
        val CONFIG_FILES = arrayOf("package.json", "tsconfig.json", ".eslintrc.json")

        const val RUNTIME_URL = "git://github.com/lf-lang/reactor-ts.git"

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

    override fun getDockerGenerator(context: LFGeneratorContext?): DockerGenerator {
        return TSDockerGenerator(context)
    }

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param context The context of this build.
     */
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        // Register the after delay transformation to be applied by GeneratorBase.
        registerTransformation(DelayedConnectionTransformation(TSDelayBodyGenerator, targetTypes, resource, true, true))

        super.doGenerate(resource, context)

        instantiationGraph

        if (!canGenerate(errorsOccurred(), mainDef, messageReporter, context)) return
        if (!isOsCompatible()) return

        // createMainReactorInstance()

        clean(context)
        copyConfigFiles()
        updatePackageConfig(context)

        val codeMaps = HashMap<Path, CodeMap>()
        generateCode(codeMaps, resource.model.preambles)
        if (targetConfig.get(DockerProperty.INSTANCE).enabled) {
            val dockerData = TSDockerGenerator(context).generateDockerData();
            dockerData.writeDockerFile()
            DockerComposeGenerator(context).writeDockerComposeFile(listOf(dockerData))
        }
        // For small programs, everything up until this point is virtually instantaneous. This is the point where cancellation,
        // progress reporting, and IDE responsiveness become real considerations.

        if (context.mode != LFGeneratorContext.Mode.LSP_MEDIUM && targetConfig.get(NoCompileProperty.INSTANCE)) {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, null))
        } else {
            context.reportProgress(
                "Code generation complete. Collecting dependencies...", IntegratedBuilder.GENERATED_PERCENT_PROGRESS
            )
            if (shouldCollectDependencies(context)) collectDependencies(resource, context, fileConfig.srcGenPkgPath, false)
            if (errorsOccurred()) {
                context.unsuccessfulFinish()
                return
            }
            if (targetConfig.get(ProtobufsProperty.INSTANCE).size != 0) {
                protoc()
            } else {
                println("No .proto files have been imported. Skipping protocol buffer compilation.")
            }
            val parsingContext = SubContext(context, COLLECTED_DEPENDENCIES_PERCENT_PROGRESS, 100)
            val validator = TSValidator(fileConfig, messageReporter, codeMaps)
            if (!context.cancelIndicator.isCanceled) {
                if (context.mode == LFGeneratorContext.Mode.LSP_MEDIUM || targetConfig.get(NoCompileProperty.INSTANCE)) {
                    if (!passesChecks(validator, parsingContext)) {
                        context.unsuccessfulFinish();
                        return;
                    }
                    context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, codeMaps))
                } else if (targetConfig.get(DockerProperty.INSTANCE).enabled) {
                    buildUsingDocker()
                } else {
                    compile(validator, resource, parsingContext)
                    concludeCompilation(context, codeMaps)
                }
            } else {
                context.unsuccessfulFinish()
            }
        }
    }

    /**
     * Prefix the given path with a scheme if missing.
     */
    private fun formatRuntimePath(path: String): String {
        return if (path.startsWith("file:") || path.startsWith("git:") || path.startsWith("git+")) {
            path
        } else {
            "file:/$path"
        }
    }

    /**
     * Update package.json according to given build parameters.
     */
    private fun updatePackageConfig(context: LFGeneratorContext) {
        var rtUri = context.args.externalRuntimeUri
        val rtVersion = context.targetConfig.get(RuntimeVersionProperty.INSTANCE)
        val sb = StringBuffer("");
        val manifest = fileConfig.srcGenPath.resolve("package.json");
        val rtRegex = Regex("(\"@lf-lang/reactor-ts\")(.+)")
        if (rtUri != null || !Strings.isNullOrEmpty(rtVersion)) {
            devMode = true;
        }
        manifest.toFile().forEachLine {
            var line = it.replace("\"LinguaFrancaDefault\"", "\"${fileConfig.name}\"");
            if (line.contains(rtRegex) && line.contains(RUNTIME_URL)) {
                devMode = true;
            }
            if (rtUri != null) {
                line = line.replace(rtRegex, "$1: \"$rtUri\",")
            } else if (!Strings.isNullOrEmpty(rtVersion)) {
                line = line.replace(rtRegex, "$1: \"$RUNTIME_URL#$rtVersion\",")
            }
            sb.appendLine(line)
        }
        manifest.toFile().writeText(sb.toString());
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
     * For each configuration file that is not present in the same directory
     * as the source file, copy a default version from $LIB_PATH/.
     */
    private fun copyConfigFiles() {
        FileUtil.copyFromClassPath(LIB_PATH, fileConfig.srcGenPath, true, true)
        for (configFile in CONFIG_FILES) {
            val override = FileUtil.findAndCopyFile(configFile, fileConfig.srcGenPath, fileConfig);
            if (override != null) {
                messageReporter.nowhere().info("Using user-provided '" + override + "'");
            } else {
                System.out.println("Using default '" + configFile + "'");
            }
        }
    }


    /**
     * Generate the code corresponding to [federate], recording any resulting mappings in [codeMaps].
     */
    private fun generateCode(
        codeMaps: MutableMap<Path, CodeMap>,
        preambles: List<Preamble>
    ) {
        val tsFileName = fileConfig.name

        val tsFilePath = fileConfig.srcGenPath.resolve("src").resolve("$tsFileName.ts")

        val tsCode = StringBuilder()

        val preambleGenerator = TSImportPreambleGenerator(fileConfig.srcFile,
            targetConfig.get(ProtobufsProperty.INSTANCE), preambles)
        tsCode.append(preambleGenerator.generatePreamble())

        val parameterGenerator = TSParameterPreambleGenerator(fileConfig, targetConfig, reactors)
        val (mainParameters, parameterCode) = parameterGenerator.generateParameters()
        tsCode.append(parameterCode)

        val reactorGenerator = TSReactorGenerator(this, messageReporter, targetConfig)
        for (reactor in reactors) {
            tsCode.append(reactorGenerator.generateReactor(reactor))
        }

        tsCode.append(reactorGenerator.generateMainReactorInstanceAndStart(this.mainDef, mainParameters))

        val codeMap = CodeMap.fromGeneratedCode(tsCode.toString())
        codeMaps[tsFilePath] = codeMap
        FileUtil.writeToFile(codeMap.generatedCode, tsFilePath)

    }

    private fun compile(validator: TSValidator, resource: Resource, parsingContext: LFGeneratorContext) {

        GeneratorUtils.refreshProject(resource, parsingContext.mode)

        if (parsingContext.cancelIndicator.isCanceled) return
        parsingContext.reportProgress("Transpiling to JavaScript...", 70)
        transpile(validator, parsingContext.cancelIndicator)

        if (parsingContext.cancelIndicator.isCanceled) return
    }

    /**
     * Return whether it is advisable to install dependencies.
     */
    private fun shouldCollectDependencies(context: LFGeneratorContext): Boolean {
        if (targetConfig.get(NoCompileProperty.INSTANCE) || targetConfig.get(DockerProperty.INSTANCE).enabled) {
            return false;
        }
        return ((context.mode != LFGeneratorContext.Mode.LSP_MEDIUM && !targetConfig.get(NoCompileProperty.INSTANCE)
                    || !fileConfig.srcGenPkgPath.resolve("node_modules").toFile().exists()))
    }

    /**
     * Collect the dependencies in package.json and their
     * transitive dependencies.
     * @param resource The Lingua Franca source file at
     * which to report any errors
     * @param context The context of this build.
     * @param path The directory for which to get dependencies.
     * @param production Whether to get production dependencies only.
     */
    private fun collectDependencies(resource: Resource, context: LFGeneratorContext, path: Path, production: Boolean) {

        Files.createDirectories(fileConfig.srcGenPkgPath) // may throw

        val pnpmInstall = commandFactory.createCommand("pnpm", if (production) listOf("install", "--prod") else listOf("install"), path, false)

        // Attempt to use pnpm, but fall back on npm if it is not available.
        if (pnpmInstall != null) {
            val ret = pnpmInstall.run(context.cancelIndicator)
            if (ret != 0) {
                val errors: String = pnpmInstall.errors
                messageReporter.at(GeneratorUtils.findTargetDecl(resource))
                    .error("pnpm install command failed" + if (errors.isBlank()) "." else ":\n$errors")
            }
            installProtoBufsIfNeeded(true, path, context.cancelIndicator)
        } else {
            messageReporter.nowhere(
            ).warning(
                "Falling back on npm. To prevent an accumulation of replicated dependencies, " +
                        "it is highly recommended to install pnpm globally (npm install -g pnpm)."
            )

            val npmInstall = commandFactory.createCommand("npm", if (production) listOf("install", "--production") else listOf("install"), path)

            if (npmInstall == null) {
                messageReporter.nowhere().error(NO_NPM_MESSAGE)
                return
            }

            if (npmInstall.run(context.cancelIndicator) != 0) {
                messageReporter.at(GeneratorUtils.findTargetDecl(resource))
                    .error("npm install command failed: " + npmInstall.errors)
                messageReporter.at(GeneratorUtils.findTargetDecl(resource))
                    .error(
                    "npm install command failed." +
                            "\nFor installation instructions, see: https://www.npmjs.com/get-npm"
                )
                return
            }

            // If reactor-ts is pulled from GitHub and building is done using npm,
            // first build reactor-ts (pnpm does this automatically).
            if (devMode) {
                val rtPath = path.resolve("node_modules").resolve("@lf-lang").resolve("reactor-ts")
                val buildRuntime = commandFactory.createCommand("npm", listOf("run", "build"), rtPath)
                if (buildRuntime.run(context.cancelIndicator) != 0) {
                    messageReporter.at(GeneratorUtils.findTargetDecl(resource))
                        .error("Unable to build runtime in dev mode: " + buildRuntime.errors.toString())
                }
            }

            installProtoBufsIfNeeded(false, path, context.cancelIndicator)
        }
    }

    private fun installProtoBufsIfNeeded(pnpmIsAvailable: Boolean, cwd: Path, cancelIndicator: CancelIndicator) {
        if (targetConfig.get(ProtobufsProperty.INSTANCE).size != 0) {
            commandFactory.createCommand(
                if (pnpmIsAvailable) "pnpm" else "npm",
                listOf("install", "google-protobuf"),
                cwd, true
            ).run(cancelIndicator)
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
        val tsOutPath = fileConfig.srcPath.relativize(context.fileConfig.srcGenPath).resolve("src")

        protocArgs.addAll(
            listOf(
                "--plugin=protoc-gen-ts=" + fileConfig.srcGenPkgPath.resolve("node_modules").resolve(".bin").resolve("protoc-gen-ts"),
                "--js_out=import_style=commonjs,binary:$tsOutPath",
                "--ts_out=$tsOutPath"
            )
        )
        protocArgs.addAll(targetConfig.get(ProtobufsProperty.INSTANCE))
        val protoc = commandFactory.createCommand("protoc", protocArgs, fileConfig.srcPath)

        if (protoc == null) {
            messageReporter.nowhere().error("Processing .proto files requires libprotoc >= 3.6.1.")
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
            messageReporter.nowhere().error("protoc failed with error code $returnCode.")
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
    private fun transpile(validator: TSValidator, cancelIndicator: CancelIndicator) {
        println("Compiling")
        val tsc = commandFactory.createCommand("npm", listOf("run", "build"), fileConfig.srcGenPkgPath)

        if (tsc == null) {
            messageReporter.nowhere().error(NO_NPM_MESSAGE)
            return
        }

        if (validator.run(tsc, cancelIndicator) == 0) {
            println("SUCCESS (compiling generated TypeScript code)")
        } else {
            messageReporter.nowhere().error("Compiler failed with the following errors:\n${tsc.errors}")
        }
    }

    /**
     * Inform the context of the results of a compilation.
     * @param context The context of the compilation.
     */
    private fun concludeCompilation(context: LFGeneratorContext, codeMaps: Map<Path, CodeMap>) {
        if (messageReporter.errorsOccurred) {
            context.unsuccessfulFinish()
        } else {
            context.finish(GeneratorResult.Status.COMPILED, codeMaps)
            val shScriptPath = fileConfig.binPath.resolve(fileConfig.name)
            val jsPath = fileConfig.srcGenPath.resolve("dist").resolve("${fileConfig.name}.js")
            FileUtil.writeToFile("#!/bin/sh\nnode $jsPath", shScriptPath)
            shScriptPath.toFile().setExecutable(true)
            messageReporter.nowhere().info("Script for running the program: $shScriptPath.")
        }
    }

    private fun isOsCompatible(): Boolean {
        return true
    }

    override fun getTargetTypes(): TargetTypes = TSTypes.getInstance()

    override fun getTarget(): Target {
        return Target.TS
    }
}
