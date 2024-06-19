package org.lflang.generator.cpp

import org.lflang.generator.CodeMap
import org.lflang.generator.LFGeneratorContext
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.CompilerProperty
import org.lflang.target.property.type.BuildTypeType.BuildType
import org.lflang.toUnixString
import org.lflang.util.FileUtil
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import kotlin.io.path.name

/** C++ platform generator for the default native platform  without additional dependencies.*/
class CppStandaloneGenerator(generator: CppGenerator) :
    CppPlatformGenerator(generator) {

    companion object {
        fun buildTypeToCmakeConfig(type: BuildType) = when (type) {
            BuildType.TEST -> "Debug"
            else           -> type.toString()
        }
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

    override fun getBuildCommands(additionalCmakeArgs: List<String>, parallelize: Boolean): List<List<String>> {
        val cmakeCommand = createCmakeCommand(Path.of("./build"), Path.of("."), additionalCmakeArgs, "src-gen")
        val makeCommand = createMakeCommand(fileConfig.buildPath, true, fileConfig.name)
        val installCommand = createMakeCommand(Path.of("./build"), true, "install")
        return listOf(cmakeCommand, makeCommand, installCommand).map { it.command() }
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

    private fun createMakeCommand(buildPath: Path, parallelize: Boolean, target: String, ): LFCommand {
        val cmakeConfig = buildTypeToCmakeConfig(targetConfig.get(BuildTypeProperty.INSTANCE))
        val makeArgs: MutableList<String> = listOf(
            "--build",
            buildPath.name,
            "--target",
            target,
            "--config",
            cmakeConfig
        ).toMutableList()

        if (parallelize) {
            makeArgs.addAll(listOf("--parallel", Runtime.getRuntime().availableProcessors().toString()))
        }

        return commandFactory.createCommand("cmake", makeArgs, buildPath.parent)
    }

    private fun createCmakeCommand(buildPath: Path, outPath: Path, additionalCmakeArgs: List<String> = listOf(), sourcesRoot: String? = null): LFCommand {
        val cmd = commandFactory.createCommand(
            "cmake",
            cmakeArgs + additionalCmakeArgs + listOf(
                "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
                "-DCMAKE_INSTALL_BINDIR=${if (outPath.isAbsolute) outPath.relativize(fileConfig.binPath).toUnixString() else fileConfig.binPath.name}",
                "-S",
                sourcesRoot ?: fileConfig.srcGenBasePath.toUnixString(),
                "-B",
                buildPath.name
            ),
            buildPath.parent
        )

        // prepare cmake
        if (targetConfig.isSet(CompilerProperty.INSTANCE)) {
            cmd.setEnvironmentVariable("CXX", targetConfig.get(CompilerProperty.INSTANCE))
        }
        return cmd
    }
}
