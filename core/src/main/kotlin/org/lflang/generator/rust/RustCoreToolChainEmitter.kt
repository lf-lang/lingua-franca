package org.lflang.generator.rust

import org.lflang.escapeStringLiteral
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.rust.RustBoardInfo.getBoardInfo
import org.lflang.joinWithCommas
import org.lflang.joinWithLn
import org.lflang.target.property.type.BuildTypeType.BuildType
import org.lflang.withDQuotes
import java.nio.file.Paths

/**
 * Emits the build configuration of programs that run on a
 * microcontroller: `Cargo.toml`, `.cargo/config.toml` and
 * `rust-toolchain.toml`.
 */
object RustCoreToolChainEmitter : RustEmitterBase() {

    /** Runtime features that need an OS, and are dropped when building for a board. */
    private val hostedRuntimeFeatures = setOf("std", PARALLEL_RT_FEATURE)

    fun Emitter.makeCoreCargoTomlFile(gen: GenerationInfo) {
        val (crate) = gen
        val board = getBoardInfo(gen.platform)
        val userDependencies = crate.dependencies.mapValues { (name, spec) ->
            if (name == runtimeCrateFullName) spec.toRuntimeToml() else spec.toToml()
        }
        // user dependencies win over the board's, so that board crates can be pinned or patched
        val allDependencies = board.dependencies + userDependencies
        this += """
            |${generatedByComment("#")}
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "${crate.rustEdition}"
            |# Keep the features of build-time dependencies (which use std) apart from the firmware's.
            |resolver = "2"
            |
            |[dependencies]
${"         |"..allDependencies.asIterable().joinWithLn { (name, toml) -> "$name = $toml" }}
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
            |test = false
            |bench = false
            |
            |[profile.dev]
            |# Unoptimized code is too large and slow for most microcontrollers.
            |opt-level = "s"
            |
            |[profile.${BuildType.RELEASE.cargoProfileName}] # use `build-type: ${BuildType.RELEASE}`
            |lto = "fat"
            |codegen-units = 1
            |
            |[profile.${BuildType.MIN_SIZE_REL.cargoProfileName}] # use `build-type: ${BuildType.MIN_SIZE_REL}`
            |inherits = "release"
            |opt-level = "s"
            |
            |[profile.${BuildType.REL_WITH_DEB_INFO.cargoProfileName}] # use `build-type: ${BuildType.REL_WITH_DEB_INFO}`
            |inherits = "release"
            |debug = true
            |
            |[profile.${BuildType.TEST.cargoProfileName}] # use `build-type: ${BuildType.TEST}`
            |inherits = "dev"
            |debug = true
            |
        """.trimMargin()
    }

    fun Emitter.makeConfigTomlFile(gen: GenerationInfo) {
        val board = getBoardInfo(gen.platform)
        val buildStd = if (board.buildStd.isEmpty()) "" else "\n[unstable]\nbuild-std = ${board.buildStd.toTomlArray()}\n"
        this += """
            |${generatedByComment("#")}
            |[target.${board.targetTriple}]
            |runner = ${board.runner.asStringLiteral()}
            |rustflags = ${board.rustFlags.toTomlArray()}
            |
            |[build]
            |target = ${board.targetTriple.asStringLiteral()}
            |
        """.trimMargin() + buildStd
    }

    fun Emitter.makeRustToolchainToml(gen: GenerationInfo) {
        val board = getBoardInfo(gen.platform)
        val targets = if (board.toolchainTargets.isEmpty()) "" else "targets = ${board.toolchainTargets.toTomlArray()}\n"
        this += """
            |${generatedByComment("#")}
            |[toolchain]
            |channel = ${board.toolchainChannel.asStringLiteral()}
            |
        """.trimMargin() + targets
    }

    /** The runtime crate, with its `embassy` backend instead of `std`. */
    private fun CargoDependencySpec.toRuntimeToml(): String {
        val runtimeFeatures = (features - hostedRuntimeFeatures) + listOf("embassy", "vec-id-sets")
        return toToml(runtimeFeatures, defaultFeatures = false)
    }

    private fun CargoDependencySpec.toToml(
        features: Collection<String>? = this.features,
        defaultFeatures: Boolean = true,
    ): String = mutableMapOf<String, String>().apply {
        if (version != null) this["version"] = version.asStringLiteral()
        if (localPath != null) this["path"] = Paths.get(localPath).toAbsolutePath().toString().asStringLiteral()
        if (!defaultFeatures) this["default-features"] = "false"
        if (!features.isNullOrEmpty()) this["features"] = features.sorted().toTomlArray()
        if (gitRepo != null) this["git"] = gitRepo.asStringLiteral()
        if (rev != null) this["rev"] = rev.asStringLiteral()
    }.asIterable().joinWithCommas("{ ", " }", trailing = false) { (k, v) -> "$k = $v" }

    private fun String.asStringLiteral() = escapeStringLiteral().withDQuotes()

    private fun Collection<String>.toTomlArray(): String = joinToString(", ") { it.asStringLiteral() }.let { "[$it]" }
}
