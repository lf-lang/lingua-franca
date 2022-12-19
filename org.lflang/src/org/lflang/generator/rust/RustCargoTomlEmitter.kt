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
import org.lflang.joinWithLn
import org.lflang.withDQuotes
import java.nio.file.Paths

/**
 * Emits the `Cargo.toml` file at the root of the generated
 * rust project.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
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
            |clap = { version = "3.1.8", features = ["derive", "env"], optional = true }
${"         |"..crate.dependencies.asIterable().joinWithLn { (name, spec) -> name + " = " + spec.toToml() }}
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
        if (version != null) this["version"] = version.asStringLiteral()
        if (localPath != null) this["path"] = Paths.get(localPath).toAbsolutePath().toString().asStringLiteral()
        if (features != null) this["features"] = features.map { it.asStringLiteral() }.joinWithCommas("[", "]")
        if (gitRepo != null) this["git"] = gitRepo.asStringLiteral()
        if (rev != null) this["rev"] = rev.asStringLiteral()
    }.asIterable().joinWithCommas("{ ", " }", trailing = false) { (k, v) -> "$k = $v" }

    private fun String.asStringLiteral() = escapeStringLiteral().withDQuotes()


}
