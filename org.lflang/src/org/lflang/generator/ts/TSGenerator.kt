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
import org.lflang.Target
import org.lflang.TimeValue
import org.lflang.ast.AfterDelayTransformation
import org.lflang.generator.*
import org.lflang.generator.GeneratorUtils.canGenerate
import org.lflang.lf.Preamble
import org.lflang.model
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
 *  @author Matt Weber
 *  @author Edward A. Lee
 *  @author Marten Lohstroh
 *  @author Christian Menard
 *  @author Hokeun Kim
 */
class TSGenerator(
    private val context: LFGeneratorContext,
    private val scopeProvider: LFGlobalScopeProvider
) : GeneratorBase(context) {


    val fileConfig: TSFileConfig = context.fileConfig as TSFileConfig

    companion object {
        /** Path to the TS lib directory (relative to class path)  */
        const val LIB_PATH = "/lib/ts"

        /**
         * Names of the configuration files to check for and copy to the generated
         * source package root if they cannot be found in the source directory.
         */
        val CONFIG_FILES = arrayOf("package.json", "tsconfig.json", "babel.config.js", ".eslintrc.json")

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

    /** Generate TypeScript code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation.
     *  @param resource The resource containing the source code.
     *  @param context The context of this build.
     */
    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        // Register the after delay transformation to be applied by GeneratorBase.
        registerTransformation(AfterDelayTransformation(TSDelayBodyGenerator, targetTypes, resource))

        super.doGenerate(resource, context)

        instantiationGraph

        if (!canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return
        if (!isOsCompatible()) return

        // createMainReactorInstance()

        clean(context)
        copyConfigFiles()
        updatePackageConfig(context)

        val codeMaps = HashMap<Path, CodeMap>()
        generateCode(codeMaps, resource.model.preambles)
        if (targetConfig.dockerOptions != null) {
            val dockerData = TSDockerGenerator(context).generateDockerData();
            dockerData.writeDockerFile()
            DockerComposeGenerator(context).writeDockerComposeFile(listOf(dockerData))
        }
        // For small programs, everything up until this point is virtually instantaneous. This is the point where cancellation,
        // progress reporting, and IDE responsiveness become real considerations.

        if (context.mode != LFGeneratorContext.Mode.LSP_MEDIUM && targetConfig.noCompile) {
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
            if (targetConfig.protoFiles.size != 0) {
                protoc()
            } else {
                println("No .proto files have been imported. Skipping protocol buffer compilation.")
            }
            val parsingContext = SubContext(context, COLLECTED_DEPENDENCIES_PERCENT_PROGRESS, 100)
            if (
                !context.cancelIndicator.isCanceled
                && passesChecks(TSValidator(fileConfig, errorReporter, codeMaps), parsingContext)
            ) {
                if (context.mode == LFGeneratorContext.Mode.LSP_MEDIUM) {
                    context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, codeMaps))
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
        var rtPath = LFGeneratorContext.BuildParm.EXTERNAL_RUNTIME_PATH.getValue(context)
        val rtVersion = LFGeneratorContext.BuildParm.RUNTIME_VERSION.getValue(context)
        val sb = StringBuffer("");
        val manifest = fileConfig.srcGenPath.resolve("package.json");
        val rtRegex = Regex("(\"@lf-lang/reactor-ts\")(.+)")
        if (rtPath != null) rtPath = formatRuntimePath(rtPath)
        // FIXME: do better CLI arg validation upstream
        // https://github.com/lf-lang/lingua-franca/issues/1429
        manifest.toFile().forEachLine {
            var line = it.replace("\"LinguaFrancaDefault\"", "\"${fileConfig.name}\"");
            if (rtPath != null) {
                line = line.replace(rtRegex, "$1: \"$rtPath\",")
            } else if (rtVersion != null) {
                line = line.replace(rtRegex, "$1: \"git://github.com/lf-lang/reactor-ts.git#$rtVersion\",")
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
        for (configFile in CONFIG_FILES) {
            val configFileDest = fileConfig.srcGenPath.resolve(configFile)
            val configFileInSrc = fileConfig.srcPath.resolve(configFile)
            if (configFileInSrc.toFile().exists()) {
                println("Copying $configFileInSrc to $configFileDest")
                Files.createDirectories(configFileDest.parent)
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
    private fun generateCode(
        codeMaps: MutableMap<Path, CodeMap>,
        preambles: List<Preamble>
    ) {
        val tsFileName = fileConfig.name

        val tsFilePath = fileConfig.srcGenPath.resolve("src").resolve("$tsFileName.ts")

        val tsCode = StringBuilder()

        val preambleGenerator = TSImportPreambleGenerator(fileConfig.srcFile,
            targetConfig.protoFiles, preambles)
        tsCode.append(preambleGenerator.generatePreamble())

        val parameterGenerator = TSParameterPreambleGenerator(fileConfig, targetConfig, reactors)
        val (mainParameters, parameterCode) = parameterGenerator.generateParameters()
        tsCode.append(parameterCode)

        val reactorGenerator = TSReactorGenerator(this, errorReporter, targetConfig)
        for (reactor in reactors) {
            tsCode.append(reactorGenerator.generateReactor(reactor))
        }

        tsCode.append(reactorGenerator.generateMainReactorInstanceAndStart(this.mainDef, mainParameters))

        val codeMap = CodeMap.fromGeneratedCode(tsCode.toString())
        codeMaps[tsFilePath] = codeMap
        FileUtil.writeToFile(codeMap.generatedCode, tsFilePath)

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
                val errors: String = pnpmInstall.errors.toString()
                errorReporter.reportError(
                    GeneratorUtils.findTarget(resource),
                    "ERROR: pnpm install command failed" + if (errors.isBlank()) "." else ":\n$errors")
            }
            installProtoBufsIfNeeded(true, path, context.cancelIndicator)
        } else {
            errorReporter.reportWarning(
                "Falling back on npm. To prevent an accumulation of replicated dependencies, " +
                        "it is highly recommended to install pnpm globally (npm install -g pnpm).")
            val npmInstall = commandFactory.createCommand("npm", if (production) listOf("install", "--production") else listOf("install"), path)

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
            installProtoBufsIfNeeded(false, path, context.cancelIndicator)
        }
    }

    private fun installProtoBufsIfNeeded(pnpmIsAvailable: Boolean, cwd: Path, cancelIndicator: CancelIndicator) {
        if (targetConfig.protoFiles.size != 0) {
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
        protocArgs.addAll(targetConfig.protoFiles)
        val protoc = commandFactory.createCommand("protoc", protocArgs, fileConfig.srcPath)

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
            context.finish(GeneratorResult.Status.COMPILED, codeMaps)
        }
    }

    private fun isOsCompatible(): Boolean {
        return true
    }

    override fun getTargetTypes(): TargetTypes = TSTypes

    override fun getTarget(): Target {
        return Target.TS
    }
}
