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

import org.lflang.generator.CodeMap
import org.lflang.generator.PrependOperator
import org.lflang.joinWithLn
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path


/**
 * Part of the Rust generator that emits the actual Rust code,
 * including its project structure. Its entry point is
 * [generateRustProject].
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
object RustEmitter : RustEmitterBase() {

    fun generateRustProject(fileConfig: RustFileConfig, gen: GenerationInfo): Map<Path, CodeMap> {
        val codeMaps = mutableMapOf<Path, CodeMap>()
        val platformOptions = gen.platform

        if (!platformOptions.board().setByUser()) {
            fileConfig.emit(codeMaps, "Cargo.toml") {
                with(RustCargoTomlEmitter) {
                    makeCargoTomlFile(gen)
                }
            }

            // if singleFile, this file will contain every module.
            fileConfig.emit(codeMaps, "src/main.rs") {
                with(RustMainFileEmitter) {
                    makeMainFile(gen)
                }
            }
        } else {
            fileConfig.emit(codeMaps, "Cargo.toml") {
                with(RustCoreToolChainEmitter) {
                    makeCoreCargoTomlFile(gen)
                }
            }
            fileConfig.emit(codeMaps, "rust-toolchain.toml") {
                with(RustCoreToolChainEmitter) {
                    makeRustToolchainToml(gen)
                }
            }
            // if singleFile, this file will contain every module.
            fileConfig.emit(codeMaps, "src/main.rs") {
                with(RustCoreMainFileEmitter) {
                    makeCoreMainFile(gen)
                }
            }
            fileConfig.emit(codeMaps, ".cargo/config.toml") {
                with(RustCoreToolChainEmitter) {
                    makeConfigTomlFile(gen)
                }
            }
            for ((path, content) in RustBoardInfo.getBoardInfo(platformOptions).extraFiles) {
                fileConfig.emit(codeMaps, path) { this += content }
            }
        }

        if (!gen.properties.singleFile) {
            fileConfig.emit(codeMaps, "src/globals.rs") { makeGlobalsModule(gen) }
            fileConfig.emit(codeMaps, "src/reactors.rs") { makeReactorsAggregateModule(gen) }
            for (reactor in gen.reactors) {
                fileConfig.emit(codeMaps, "src/reactors/${reactor.names.modFileName}.rs") {
                    with(RustReactorEmitter) {
                        emitReactorModule(reactor)
                    }
                }
            }
        }

        // copy user-defined modules
        for (modPath in gen.crate.modulesToIncludeInMain) {
            val target = fileConfig.srcGenPath.resolve("src").resolve(modPath.fileName)
            if (Files.isDirectory(modPath)) {
                FileUtil.copyDirectoryContents(modPath, target)
            } else {
                FileUtil.copyFile(modPath, target)
            }
        }
        return codeMaps
    }

    /**
     * Contents of the `globals` module: the file-level preambles.
     * Items declared there (consts, statics, pin assignments, helper
     * functions) are visible from `main` and from every reactor.
     */
    internal fun Emitter.makeGlobalsModule(gen: GenerationInfo) {
        val runtimeSupport = if (gen.platform.board().setByUser()) embassySpawnerSupport else ""
        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![allow(unused)]
            |
            |// same names as in reactions (AsyncCtx, Asap, After, Duration, ...)
            |use $rsRuntime::prelude::*;
            |
${"         |"..runtimeSupport}
            |
${"         |"..gen.globalPreambles.joinToString("\n\n") { "// preamble {=\n${it.trimIndent()}\n// =}" }}
            |
        """.trimMargin()
        }
    }

    /**
     * Gives reactions access to the embassy executor, so they can spawn tasks
     * (e.g. ones that feed a physical action through `ctx.new_async_ctx()`).
     * `Spawner` is `!Send`, so the cell holds its `SendSpawner` instead.
     */
    private val embassySpawnerSupport = """
        |static __LF_SPAWNER: critical_section::Mutex<core::cell::Cell<Option<embassy_executor::SendSpawner>>> =
        |    critical_section::Mutex::new(core::cell::Cell::new(None));
        |
        |/// Spawner of the embassy executor that runs the reactor program.
        |/// Only spawns tasks whose arguments are `Send`.
        |pub fn spawner() -> embassy_executor::SendSpawner {
        |    critical_section::with(|cs| __LF_SPAWNER.borrow(cs).get())
        |        .expect("the spawner is set by `main` before any reaction runs")
        |}
        |
        |#[doc(hidden)]
        |pub fn __lf_set_spawner(spawner: embassy_executor::SendSpawner) {
        |    critical_section::with(|cs| __LF_SPAWNER.borrow(cs).set(Some(spawner)));
        |}
    """.trimMargin()

    private fun Emitter.makeReactorsAggregateModule(gen: GenerationInfo) {
        fun ReactorInfo.modDecl(): String = with(names) {
            // We make some declarations public to be able to refer to them
            // simply when building nested reactors.
            """
                mod $modName;
                pub use self::$modName::$wrapperName;
                pub use self::$modName::$paramStructName;
            """.trimIndent()
        }

        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |
${"         |"..gen.reactors.joinWithLn { it.modDecl() }}
            |
        """.trimMargin()
        }
    }
}
