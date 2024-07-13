package org.lflang.generator.cpp

import org.apache.commons.text.StringEscapeUtils
import org.lflang.generator.CodeMap
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.docker.DockerGenerator
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.CompilerProperty
import org.lflang.target.property.DockerProperty
import org.lflang.target.property.type.BuildTypeType.BuildType
import org.lflang.toUnixString
import org.lflang.util.FileUtil
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

/** C++ platform generator for the default native platform  without additional dependencies.*/
class CppStandaloneGenerator(generator: CppGenerator) :
    CppPlatformGenerator(generator) {

    companion object {
        fun buildTypeToCmakeConfig(type: BuildType) = when (type) {
            BuildType.TEST -> "Debug"
            else           -> type.toString()
        }

        const val DEFAULT_BASE_IMAGE: String = "alpine:latest"
    }

    override fun generatePlatformFiles() {

        // generate the main source file (containing main())
        val mainFile = Paths.get("main.cc")
        val mainCodeMap =
            CodeMap.fromGeneratedCode(
                CppStandaloneMainGenerator(
                    mainReactor,
                    generator.targetConfig,
                    generator.fileConfig
                ).generateCode()
            )
        cppSources.add(mainFile)
        codeMaps[fileConfig.srcGenPath.resolve(mainFile)] = mainCodeMap
        println("Path: $srcGenPath $srcGenPath")

        FileUtil.writeToFile(mainCodeMap.generatedCode, srcGenPath.resolve(mainFile), true)

        // generate the cmake scripts
        val cmakeGenerator = CppStandaloneCmakeGenerator(targetConfig, generator.fileConfig)
        val srcGenRoot = fileConfig.srcGenBasePath
        val pkgName = fileConfig.srcGenPkgPath.fileName.toString()
        FileUtil.writeToFile(cmakeGenerator.generateRootCmake(pkgName), srcGenRoot.resolve("CMakeLists.txt"), true)
        FileUtil.writeToFile(cmakeGenerator.generateCmake(cppSources), srcGenPath.resolve("CMakeLists.txt"), true)
        FileUtil.writeToFile("", srcGenPath.resolve(".lf-cpp-marker"), true)
        var subdir = srcGenPath.parent
        while (subdir != srcGenRoot) {
            FileUtil.writeToFile(cmakeGenerator.generateSubdirCmake(), subdir.resolve("CMakeLists.txt"), true)
            FileUtil.writeToFile("", subdir.resolve(".lf-cpp-marker"), true)
            subdir = subdir.parent
        }
    }

    override fun doCompile(context: LFGeneratorContext, onlyGenerateBuildFiles: Boolean): Boolean {
        var runMake = !onlyGenerateBuildFiles
        if (onlyGenerateBuildFiles && !fileConfig.cppBuildDirectories.all { it.toFile().exists() }) {
            // Special case: Some build directories do not exist, perhaps because this is the first C++ validation
            //  that has been done in this LF package since the last time the package was cleaned.
            //  We must compile in order to install the dependencies. Future validations will be faster.
            runMake = true
        }

        // make sure the build directory exists
        Files.createDirectories(fileConfig.buildPath)

        val version = checkCmakeVersion()
        var parallelize = true
        if (version != null && version.compareVersion("3.12.0") < 0) {
            messageReporter.nowhere().warning("CMAKE is older than version 3.12. Parallel building is not supported.")
            parallelize = false
        }

        if (version != null) {
            val cmakeReturnCode = runCmake(context)

            if (cmakeReturnCode == 0 && runMake) {
                // If cmake succeeded, run make
                val makeCommand = createMakeCommand(fileConfig.buildPath, parallelize, fileConfig.name)
                val makeReturnCode = CppValidator(fileConfig, messageReporter, codeMaps).run(makeCommand, context.cancelIndicator)
                var installReturnCode = 0
                if (makeReturnCode == 0) {
                    val installCommand = createMakeCommand(fileConfig.buildPath, parallelize, "install")
                    installReturnCode = installCommand.run(context.cancelIndicator)
                    if (installReturnCode == 0) {
                        println("SUCCESS (compiling generated C++ code)")
                        println("Generated source code is in ${fileConfig.srcGenPath}")
                        println("Compiled binary is in ${fileConfig.binPath}")
                    }
                }
                if ((makeReturnCode != 0 || installReturnCode != 0) && !messageReporter.errorsOccurred) {
                    // If errors occurred but none were reported, then the following message is the best we can do.
                    messageReporter.nowhere().error("make failed with error code $makeReturnCode")
                }
            }
            if (cmakeReturnCode != 0) {
                messageReporter.nowhere().error("cmake failed with error code $cmakeReturnCode")
            }
        }
        return !messageReporter.errorsOccurred
    }

    private fun checkCmakeVersion(): String? {
        // get the installed cmake version and make sure it is at least 3.5
        val cmd = commandFactory.createCommand("cmake", listOf("--version"), fileConfig.buildPath)
        var version: String? = null
        if (cmd != null && cmd.run() == 0) {
            val regex = "\\d+(\\.\\d+)+".toRegex()
            version = regex.find(cmd.output.toString())?.value
        }
        if (version == null || version.compareVersion("3.5.0") < 0) {
            messageReporter.nowhere(
            ).error(
                "The C++ target requires CMAKE >= 3.5.0 to compile the generated code. " +
                        "Auto-compiling can be disabled using the \"no-compile: true\" target property."
            )
            return null
        }

        return version
    }


    /**
     * Run CMake to generate build files.
     * @return True, if cmake run successfully
     */
    private fun runCmake(context: LFGeneratorContext): Int {
        val cmakeCommand = createCmakeCommand(fileConfig.buildPath, fileConfig.outPath)
        return cmakeCommand.run(context.cancelIndicator)
    }

    private fun String.compareVersion(other: String): Int {
        val a = this.split(".").map { it.toInt() }
        val b = other.split(".").map { it.toInt() }
        for (x in (a zip b)) {
            val res = x.first.compareTo(x.second)
            if (res != 0)
                return res
        }
        return 0
    }

    private fun getMakeArgs(buildPath: Path, parallelize: Boolean, target: String): List<String> {
        val cmakeConfig = buildTypeToCmakeConfig(targetConfig.get(BuildTypeProperty.INSTANCE))
        val makeArgs = mutableListOf(
            "--build",
            buildPath.fileName.toString(),
            "--target",
            target,
            "--config",
            cmakeConfig
        )

        if (parallelize) {
            makeArgs.addAll(listOf("--parallel", Runtime.getRuntime().availableProcessors().toString()))
        }

        return makeArgs
    }


    private fun createMakeCommand(buildPath: Path, parallelize: Boolean, target: String): LFCommand {
        val makeArgs = getMakeArgs(buildPath, parallelize, target)
        return commandFactory.createCommand("cmake", makeArgs, buildPath.parent)
    }

    private fun getCmakeArgs(
        buildPath: Path,
        outPath: Path,
        additionalCmakeArgs: List<String> = listOf(),
        sourcesRoot: String? = null
    ) = cmakeArgs + additionalCmakeArgs + listOf(
        "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
        "-DCMAKE_INSTALL_BINDIR=$relativeBinDir",
        "-S",
        sourcesRoot ?: fileConfig.srcGenBasePath.toUnixString(),
        "-B",
        buildPath.fileName.toString()
    )

    private fun createCmakeCommand(
        buildPath: Path,
        outPath: Path,
        additionalCmakeArgs: List<String> = listOf(),
        sourcesRoot: String? = null
    ): LFCommand {
        val cmd = commandFactory.createCommand(
            "cmake",
            getCmakeArgs(buildPath, outPath, additionalCmakeArgs, sourcesRoot),
            buildPath.parent
        )

        // prepare cmake
        if (targetConfig.isSet(CompilerProperty.INSTANCE)) {
            cmd.setEnvironmentVariable("CXX", targetConfig.get(CompilerProperty.INSTANCE))
        }
        return cmd
    }


    inner class StandaloneDockerGenerator(context: LFGeneratorContext?) : DockerGenerator(context) {

        override fun generateCopyForSources(): String = """
                COPY src src
                COPY src-gen src-gen
            """.trimIndent()

        override fun defaultImage(): String = DEFAULT_BASE_IMAGE

        override fun generateRunForInstallingDeps(): String {
            return if (builderBase() == defaultImage()) {
                ("RUN set -ex && apk add --no-cache g++ musl-dev cmake make && apk add --no-cache"
                        + " --update --repository=https://dl-cdn.alpinelinux.org/alpine/v3.16/main/"
                        + " libexecinfo-dev")
            } else {
                "# (Skipping installation of build dependencies; custom base image.)"
            }
        }

        override fun defaultEntryPoint(): List<String> = listOf("$relativeBinDir/${fileConfig.name}")

        override fun generateCopyOfExecutable(): String {
            val name = fileConfig.name
            return """
                COPY --from=builder /lingua-franca/$name/$relativeBinDir/$name ./$relativeBinDir/$name
                COPY --from=builder /usr/local/lib /usr/local/lib
                COPY --from=builder /usr/lib /usr/lib
                COPY --from=builder /lingua-franca .
            """.trimIndent()
        }

        override fun defaultBuildCommands(): List<String> {
            val mkdirCommand = listOf("mkdir", "-p", "build")
            val commands = listOf(
                mkdirCommand,
                listOf("cmake") + getCmakeArgs(
                    Path.of("./build"),
                    Path.of("."),
                    listOf("-DREACTOR_CPP_LINK_EXECINFO=ON"),
                    "src-gen"
                ),
                listOf("cmake") + getMakeArgs(fileConfig.buildPath, true, fileConfig.name),
                listOf("cmake") + getMakeArgs(Path.of("./build"), true, "install")
            )
            return commands.map { argListToCommand(it) }
        }

        override fun getPreBuildCommand(): MutableList<String> {
            val script = context.targetConfig.get(DockerProperty.INSTANCE).preBuildScript
            if (script.isNotEmpty()) {
                return mutableListOf(". src/" + StringEscapeUtils.escapeXSI(script))
            }
            return mutableListOf()
        }

        override fun getPostBuildCommand(): MutableList<String> {
            val script = context.targetConfig.get(DockerProperty.INSTANCE).postBuildScript
            if (script.isNotEmpty()) {
                return mutableListOf(". src/" + StringEscapeUtils.escapeXSI(script))
            }
            return mutableListOf()
        }
    }

    override fun getDockerGenerator(context: LFGeneratorContext?): DockerGenerator = StandaloneDockerGenerator(context)
}
