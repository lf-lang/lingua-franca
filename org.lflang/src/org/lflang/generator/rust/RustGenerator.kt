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
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.lflang.ErrorReporter
import org.lflang.Target
import org.lflang.TargetConfig
import org.lflang.TargetProperty.BuildType
import org.lflang.generator.canGenerate
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorBase
import org.lflang.generator.GeneratorResult
import org.lflang.generator.IntegratedBuilder
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.TargetTypes
import org.lflang.joinWithCommas
import org.lflang.lf.Action
import org.lflang.lf.VarRef
import org.lflang.scoping.LFGlobalScopeProvider
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
    fileConfig: RustFileConfig,
    errorReporter: ErrorReporter,
    @Suppress("UNUSED_PARAMETER") unused: LFGlobalScopeProvider
) : GeneratorBase(fileConfig, errorReporter) {

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: LFGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        if (!canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return

        val fileConfig = fileConfig as RustFileConfig

        Files.createDirectories(fileConfig.srcGenPath)

        val gen = RustModelBuilder.makeGenerationInfo(targetConfig, reactors)
        val codeMaps: Map<Path, CodeMap> = RustEmitter.generateRustProject(fileConfig, gen)

        if (targetConfig.noCompile || errorsOccurred()) {
            context.finish(GeneratorResult.GENERATED_NO_EXECUTABLE.apply(codeMaps))
            println("Exiting before invoking target compiler.")
        } else {
            context.reportProgress(
                "Code generation complete. Compiling...", IntegratedBuilder.GENERATED_PERCENT_PROGRESS
            )
            val exec = fileConfig.binPath.toAbsolutePath().resolve(gen.executableName)
            Files.deleteIfExists(exec) // cleanup, cargo doesn't do it
            if (context.mode == TargetConfig.Mode.LSP_MEDIUM) RustValidator(fileConfig, errorReporter, codeMaps).doValidate(context.cancelIndicator)
            else invokeRustCompiler(context, gen.executableName, codeMaps)
        }
    }

    private fun invokeRustCompiler(context: LFGeneratorContext, executableName: String, codeMaps: Map<Path, CodeMap>) {

        val args = mutableListOf<String>().apply {
            this += listOf(
                "+nightly",
                "build",
                // note that this option is unstable for now and requires rust nightly ...
                "--out-dir", fileConfig.binPath.toAbsolutePath().toString(),
                "-Z", "unstable-options", // ... and that feature flag
            )

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

            if (context.mode != TargetConfig.Mode.STANDALONE) {
                this += listOf("--message-format", "json")
            }
        }

        val cargoCommand = commandFactory.createCommand(
            "cargo", args,
            fileConfig.srcGenPath.toAbsolutePath()
        ) ?: return

        val cargoReturnCode = RustValidator(fileConfig, errorReporter, codeMaps).run(cargoCommand, context.cancelIndicator)

        if (cargoReturnCode == 0) {
            println("SUCCESS (compiling generated Rust code)")
            println("Generated source code is in ${fileConfig.srcGenPath}")
            println("Compiled binary is in ${fileConfig.binPath}")
            context.finish(GeneratorResult.Status.COMPILED, executableName, fileConfig, codeMaps)
        } else if (context.cancelIndicator.isCanceled) {
            context.finish(GeneratorResult.CANCELLED)
        } else {
            if (!errorsOccurred()) errorReporter.reportError("cargo failed with error code $cargoReturnCode")
            context.finish(GeneratorResult.FAILED)
        }
    }


    override fun getTarget(): Target = Target.Rust

    override fun getTargetTypes(): TargetTypes = RustTypes

    override fun generateDelayBody(action: Action, port: VarRef): String {
        TODO("Not yet implemented")
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        TODO("Not yet implemented")
    }

    override fun generateDelayGeneric(): String {
        TODO("Not yet implemented")
    }

}
