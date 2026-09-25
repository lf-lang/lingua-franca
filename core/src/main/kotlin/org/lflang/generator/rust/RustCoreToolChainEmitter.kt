package org.lflang.generator.rust

import org.lflang.escapeStringLiteral
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.rust.RustBoardInfo.getBoardInfo
import org.lflang.joinWithCommas
import org.lflang.joinWithLn
import org.lflang.target.property.type.BuildTypeType.BuildType
import org.lflang.withDQuotes
import java.nio.file.Paths

object RustCoreToolChainEmitter : RustEmitterBase() {
    fun Emitter.makeCoreCargoTomlFile(gen: GenerationInfo) {
        val (crate) = gen
        val allDependencies = getAllDependencies(gen)
        this += """
            |${generatedByComment("#")}
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "${crate.rustEdition}"
            |
            |[dependencies]
${"         |"..allDependencies.asIterable().joinWithLn { (name, spec) -> name + " = " + spec.toToml() }}
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
            |
            |[profile.${BuildType.RELEASE.cargoProfileName}] # use `build-type: ${BuildType.RELEASE}`
            |lto = "thin"
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
        val (crate) = gen
        val boardInfo = getBoardInfo(gen.platform)
        this += """
            |${generatedByComment("#")}
            |[target.${boardInfo.targetTriple}]
            |runner = ${boardInfo.runner.asStringLiteral()}
            |rustflags = ${boardInfo.rustFlags.toTomlArray()}
            |
            |[build]
            |target = ${boardInfo.targetTriple.asStringLiteral()}
            |
            |[unstable]
            |build-std = ["core", "alloc"]
            |
        """.trimMargin()
    }

    fun Emitter.makeRustToolchainToml(gen: GenerationInfo) {
        val boardInfo = getBoardInfo(gen.platform)
        this += """
            |${generatedByComment("#")}
            |[toolchain]
            |channel = ${boardInfo.rustToolChainChannel.asStringLiteral()}
            |targets = [${boardInfo.targetTriple.asStringLiteral()}]
        """.trimMargin()
    }

    private fun getAllDependencies(gen: GenerationInfo): Map<String, CargoDependencySpec> {
        val boardInfo = getBoardInfo(gen.platform)
        return gen.crate.dependencies + boardInfo.dependencies
    }

    private fun CargoDependencySpec.toToml(): String = mutableMapOf<String, String>().apply {
        if (version != null) this["version"] = version.asStringLiteral()
        if (localPath != null) this["path"] = Paths.get(localPath).toAbsolutePath().toString().asStringLiteral()
        if (features != null) this["features"] = features.map { it.asStringLiteral() }.joinWithCommas("[", "]")
        if (gitRepo != null) this["git"] = gitRepo.asStringLiteral()
        if (rev != null) this["rev"] = rev.asStringLiteral()
    }.asIterable().joinWithCommas("{ ", " }", trailing = false) { (k, v) -> "$k = $v" }

    private fun String.asStringLiteral() = escapeStringLiteral().withDQuotes()

    private fun List<String>.toTomlArray(): String = joinToString(", ") { it.withDQuotes() }.let { "[$it]" }

}
