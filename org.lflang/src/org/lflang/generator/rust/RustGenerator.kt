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
import org.lflang.*
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.generator.cpp.name
import org.lflang.generator.cpp.targetType
import org.lflang.lf.Action
import org.lflang.lf.Reaction
import org.lflang.lf.VarRef

/**
 * Generates Rust code.
 */
class RustGenerator(fileConfig: RustFileConfig, errorReporter: ErrorReporter) : GeneratorBase(fileConfig, errorReporter) {

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (errorsOccurred()) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
            return
        }

        fileConfig.srcGenPath.createDirectories()

        val gen = makeGenerationInfo(resource, fsa, context)
        RustEmitter.generateFiles(fileConfig as RustFileConfig, gen)

        if (targetConfig.noCompile || errorsOccurred()) {
            println("Exiting before invoking target compiler.")
        } else {
            invokeRustCompiler()
        }
    }

    private fun makeGenerationInfo(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext): GenerationInfo {
        val reactors = makeReactorInfos()
        // todo how do we pick the main reactor? it seems like super.doGenerate sets that field...
        val mainReactor = reactors.lastOrNull { it.isMain } ?: reactors.last()

        return GenerationInfo(
            crate = CrateInfo(
                name = mainReactor.lfName.camelToSnakeCase(),
                version = "1.0.0",
                authors = listOf(System.getProperty("user.name"))
            ),
            reactors = reactors,
            mainReactor = mainReactor,
            // Rust exec names are snake case, otherwise we get a cargo warning
            // https://github.com/rust-lang/rust/issues/45127
            executableName = mainReactor.lfName.camelToSnakeCase()
        )
    }

    private fun makeReactorInfos() = reactors.map { reactor ->
        val components = mutableMapOf<String, ReactorComponent>()
        for (component in reactor.allOutputs + reactor.allInputs + reactor.allActions) {
            val irObj = ReactorComponent.from(component) ?: continue
            components[irObj.name] = irObj
        }

        val reactions = reactor.reactions.map { n: Reaction ->
            val dependencies = (n.effects + n.sources).mapTo(LinkedHashSet()) { components[it.name]!! }
            ReactionInfo(
                idx = n.indexInContainer,
                depends = dependencies,
                body = n.code.toText(),
                isStartup = n.triggers.any { it.isStartup },
                isShutdown = n.triggers.any { it.isShutdown }
            )
        }

        ReactorInfo(
            lfName = reactor.name,
            reactions = reactions,
            otherComponents = components.values.toList(),
            isMain = reactor.isMain,
            preambles = reactor.preambles.map { it.code.toText() },
            stateVars = reactor.stateVars.map { StateVarInfo(it.name, it.targetType, init = it.init.singleOrNull()?.toText()) }
        )
    }


    private fun invokeRustCompiler() {
        val cargoBuilder = createCommand(
            "cargo", listOf(
                "+nightly",
                "build",
                "--release",
                // note that this option is unstable for now and requires rust nightly ...
                "--out-dir", fileConfig.binPath.toAbsolutePath().toString(),
                "-Z", "unstable-options" // ... and that feature flag
            ),
            fileConfig.srcGenPath,
            "The Rust target requires Cargo in the path. " +
                    "Auto-compiling can be disabled using the \"no-compile: true\" target property.",
            true
        ) ?: return

        val cargoReturnCode = executeCommand(cargoBuilder)

        if (cargoReturnCode == 0) {
            println("SUCCESS (compiling generated Rust code)")
            println("Generated source code is in ${fileConfig.srcGenPath}")
            println("Compiled binary is in ${fileConfig.binPath}")
        } else {
            errorReporter.reportError("cargo failed with error code $cargoReturnCode")
        }
    }



    override fun supportsGenerics(): Boolean = true

    override fun getTargetTimeType(): String = "LogicalTime"

    override fun getTargetTagType(): String = "Tag"

    override fun getTargetTagIntervalType(): String = "Duration"

    override fun getTargetUndefinedType(): String = TODO("what's that")

    override fun getTargetFixedSizeListType(baseType: String, size: Int): String =
        "[ $baseType ; $size ]"

    override fun getTargetVariableSizeListType(baseType: String): String =
        "Vec<$baseType>"

    override fun getTarget(): Target = Target.Rust


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
