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

import org.lflang.FileConfig
import org.lflang.Target
import org.lflang.generator.rust.RustEmitter.makeReactorModule
import org.lflang.lf.Action
import org.lflang.lf.VarRef
import org.lflang.withDQuotes

/**
 * Generates Rust code
 */
object RustEmitter {

    fun generateFiles(fileConfig: RustFileConfig, gen: GenerationInfo) {

        fileConfig.emit("Cargo.toml") { makeCargoFile(gen) }
        fileConfig.emit("src/bin/main.rs") { makeMainFile(gen) }
        fileConfig.emit("src/lib.rs") { makeMainFile(gen) }
        for (reactor in gen.reactors) {
            fileConfig.emit("src/${gen.crate.name}/${reactor.modName}.rs") {
                makeReactorModule(reactor)
            }
        }

    }

    private fun Emitter.makeReactorModule(reactor: ReactorInfo) {
        val out = this
        with(reactor) {
            out += """
            |
            |struct $structName { 
            |    // state vars
            |}
            |
            |
            |struct $dispatcherName {
            |    _impl: $structName,
            |    // actions & components
            |}
            |
            |
            |reaction_ids!(
            |  ${reactions.joinToString(", ", "enum $structName Reactions {", "}") { it.rustId } }
            | );
            |
            |impl ReactorDispatcher for $dispatcherName {
            |    type ReactionId = $reactionIdName;
            |    type Wrapped = $structName;
            |    type Params = $ctorParamTypes;
            |
            |}
        """.trimMargin()
        }
    }

    private fun Emitter.makeMainFile(gen: GenerationInfo) {
        this += """
            |
            |fn main() {
            |
            |
            |}
        """.trimIndent()
    }

    private fun Emitter.makeCargoFile(gen: GenerationInfo) {
        val (crate, runtime) = gen
        this += """
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "2018"

            |# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

            |[dependencies]
            |reactor-rust = { ${runtime.toml_spec} }
        """
    }


}
