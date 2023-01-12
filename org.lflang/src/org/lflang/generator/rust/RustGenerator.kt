/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.rust

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.ErrorReporter
import org.lflang.Target
import org.lflang.TargetProperty.BuildType
import org.lflang.ast.AfterDelayTransformation
import org.lflang.generator.GeneratorUtils.canGenerate
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorBase
import org.lflang.generator.GeneratorResult
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.TargetTypes

import org.lflang.joinWithCommas
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path

/**
 * Generator for the Rust target language. The generation is
 * organized around a sort of intermediate representation
 * that is constructed from the AST in a first step, then
 * handed off to the [RustEmitter]. This makes the emitter
 * simpler, as it takes fewer design decisions. This is also
 * meant to make it easier to port the emitter to a bunch of
 * templates written in a template language like Velocity.
 *
 * See [GenerationInfo] for the main model class.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
@Suppress("unused")
class RustGenerator(
    context: LFGeneratorContext,
    @Suppress("UNUSED_PARAMETER") unused: LFGlobalScopeProvider
) : GeneratorBase(context) {

    val fileConfig: RustFileConfig = context.fileConfig as RustFileConfig

    companion object {
        /** Path to the rust runtime library (relative to class path)  */
        const val runtimeDir = "/lib/rs/reactor-rs"
        const val runtimeName = "reactor-rs"
    }

    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)

        if (!canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return

        Files.createDirectories(fileConfig.srcGenPath)

        val gen = RustModelBuilder.makeGenerationInfo(targetConfig, reactors, errorReporter)
        val codeMaps: Map<Path, CodeMap> = RustEmitter.generateRustProject(fileConfig, gen)

        if (targetConfig.noCompile || errorsOccurred()) {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(context, codeMaps))
            println("Exiting before invoking target compiler.")
        } else {
            context.reportProgress(
                "Code generation complete. Compiling...", IntegratedBuilder.GENERATED_PERCENT_PROGRESS
            )
            Files.deleteIfExists(fileConfig.executable) // cleanup, cargo doesn't do it
            if (context.mode == LFGeneratorContext.Mode.LSP_MEDIUM) RustValidator(fileConfig, errorReporter, codeMaps).doValidate(context)
            else invokeRustCompiler(context, codeMaps)
        }
    }

    private fun invokeRustCompiler(context: LFGeneratorContext, codeMaps: Map<Path, CodeMap>) {

        val args = mutableListOf<String>().apply {
            this += "build"

            val buildType = targetConfig.rust.buildType
            if (buildType == BuildType.RELEASE) {
                this += "--release"
            } else if (buildType != BuildType.DEBUG) {
                this += "--profile"
                this += buildType.cargoProfileName
            }

            if (targetConfig.rust.cargoFeatures.isNotEmpty()) {
                this += "--features"
                this += targetConfig.rust.cargoFeatures.joinWithCommas()
            }

            this += targetConfig.compilerFlags

            this += listOf("--message-format", "json-diagnostic-rendered-ansi")
        }

        val cargoCommand = commandFactory.createCommand(
            "cargo", args,
            fileConfig.srcGenPath.toAbsolutePath()
        ) ?: return
        cargoCommand.setQuiet()

        val validator = RustValidator(fileConfig, errorReporter, codeMaps)
        val cargoReturnCode = validator.run(cargoCommand, context.cancelIndicator)

        if (cargoReturnCode == 0) {
            // We still have to copy the compiled binary to the destination folder.
            val buildType = targetConfig.rust.buildType
            val binaryPath = validator.getMetadata()?.targetDirectory!!
                .resolve(buildType.cargoProfileName)
                .resolve(fileConfig.executable.fileName)
            val destPath = fileConfig.executable

            FileUtil.copyFile(binaryPath, destPath)
            // Files do not retain permissions when copied.
            destPath.toFile().setExecutable(true)

            println("SUCCESS (compiling generated Rust code)")
            println("Generated source code is in ${fileConfig.srcGenPath}")
            println("Compiled binary is in ${fileConfig.binPath}")
            context.finish(GeneratorResult.Status.COMPILED, codeMaps)
        } else if (context.cancelIndicator.isCanceled) {
            context.finish(GeneratorResult.CANCELLED)
        } else {
            if (!errorsOccurred()) errorReporter.reportError(
                "cargo failed with error code $cargoReturnCode and reported the following error(s):\n${cargoCommand.errors}"
            )
            context.finish(GeneratorResult.FAILED)
        }
    }

    override fun getTarget(): Target = Target.Rust

    override fun getTargetTypes(): TargetTypes = RustTypes

}
