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

import org.lflang.TargetProperty.BuildType.*
import org.lflang.escapeStringLiteral
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.joinWithCommas
import org.lflang.withDQuotes
import java.nio.file.Paths

/**
 * Emits the `Cargo.toml` file at the root of the generated
 * rust project.
 *
 * @author Clément Fournier
 */
object RustCargoTomlEmitter : RustEmitterBase() {
    fun Emitter.makeCargoTomlFile(gen: GenerationInfo) {
        val (crate) = gen
        this += """
            |${generatedByComment("#")}
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "2018"
            |
            |[dependencies]
            |env_logger = "0.9"
            |log = { version = "0.4", features = ["release_max_level_info"] }
            |assert_matches = {version = "1", optional = true}
            |clap = {version = "=3.0.0-beta.5", optional = true}
${"         |"..crate.dependencies.asIterable().joinToString("\n") { (name, spec) -> name + " = " + spec.toToml() }}
            |
            |[dependencies.$runtimeCrateFullName] # the reactor runtime
            |${gen.runtime.runtimeCrateSpec()}
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
            |
            |[features]
            |cli=["clap"]
            |
            |[profile.${RELEASE.cargoProfileName}] # use `build-type: $RELEASE`
            |lto = "thin"
            |codegen-units = 1
            |
            |[profile.${MIN_SIZE_REL.cargoProfileName}] # use `build-type: $MIN_SIZE_REL`
            |inherits = "release"
            |opt-level = "s"
            |
            |[profile.${REL_WITH_DEB_INFO.cargoProfileName}] # use `build-type: $REL_WITH_DEB_INFO`
            |inherits = "release"
            |debug = true
            |
        """.trimMargin()
    }


    private fun CargoDependencySpec.toToml(): String = mutableMapOf<String, String>().apply {
        if (version != null) this["version"] = version.escapeStringLiteral()
        if (localPath != null) this["path"] = localPath.escapeStringLiteral()
    }.asIterable().joinWithCommas("{ ", " }", trailing = false) { (k, v) -> "$k = \"$v\"" }


    private fun RuntimeInfo.runtimeCrateSpec(): String =
        buildString {
            if (localPath != null) {
                val localPath = Paths.get(localPath).toAbsolutePath().toString().escapeStringLiteral()

                appendLine("path = \"$localPath\"")
            } else {
                appendLine("git = \"https://github.com/lf-lang/reactor-rust.git\"")
                if (version != null) {
                    // Version just checks out a relevant tag, while we're not published to crates.io
                    // Maybe we'll never need to be published.
                    appendLine("tag=\"v$version\"")
                } else {
                    appendLine("rev=\"$gitRevision\"")
                }
            }
        }

}
