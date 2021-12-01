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
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.ErrorReporter
import org.lflang.Target
import org.lflang.TargetProperty.BuildType
import org.lflang.generator.GeneratorBase
import org.lflang.generator.TargetTypes
import org.lflang.joinWithCommas
import org.lflang.lf.*
import org.lflang.scoping.LFGlobalScopeProvider
import java.nio.file.Files

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

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (errorsOccurred()) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
            return
        }

        val fileConfig = fileConfig as RustFileConfig

        Files.createDirectories(fileConfig.srcGenPath)

        val gen = RustModelBuilder.makeGenerationInfo(targetConfig, reactors)
        RustEmitter.generateRustProject(fileConfig, gen)

        if (targetConfig.noCompile || errorsOccurred()) {
            println("Exiting before invoking target compiler.")
        } else {
            val exec = fileConfig.binPath.toAbsolutePath().resolve(gen.executableName)
            Files.deleteIfExists(exec) // cleanup, cargo doesn't do it
            invokeRustCompiler()
        }
    }

    private fun invokeRustCompiler() {

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
        }

        val cargoCommand = commandFactory.createCommand(
            "cargo", args,
            fileConfig.srcGenPath.toAbsolutePath()
        ) ?: return

        val cargoReturnCode = cargoCommand.run()

        if (cargoReturnCode == 0) {
            println("SUCCESS (compiling generated Rust code)")
            println("Generated source code is in ${fileConfig.srcGenPath}")
            println("Compiled binary is in ${fileConfig.binPath}")
        } else {
            errorReporter.reportError("cargo failed with error code $cargoReturnCode")
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

