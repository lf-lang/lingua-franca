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
import org.lflang.generator.rust.RustEmitter.generateRustProject
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

        if (!gen.properties.singleFile) {
            fileConfig.emit(codeMaps, "src/reactors/mod.rs") { makeReactorsAggregateModule(gen) }
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
                FileUtil.copyDirectory(modPath, target)
            } else {
                FileUtil.copyFile(modPath, target)
            }
        }
        return codeMaps
    }

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
