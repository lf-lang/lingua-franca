package org.lflang.generator.cpp

import org.lflang.generator.CodeMap
import org.lflang.generator.JavaGeneratorUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.toDefinition
import org.lflang.toUnixString
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

class CppStandaloneGenerator(generator: CppGenerator) :
    CppPlatformGenerator(generator) {

    override fun generatePlatformFiles() {

        // generate the main source file (containing main())
        val mainFile = Paths.get("main.cc")
        val mainCodeMap =
            CodeMap.fromGeneratedCode(CppStandaloneMainGenerator(mainReactor, generator.targetConfig, fileConfig).generateCode())
        cppSources.add(mainFile)
        codeMaps[fileConfig.srcGenPath.resolve(mainFile)] = mainCodeMap
        println("Path: $srcGenPath $srcGenPath" )

        JavaGeneratorUtils.writeToFile(mainCodeMap.generatedCode, srcGenPath.resolve(mainFile))

        // generate the cmake script
        val cmakeGenerator = CppStandaloneCmakeGenerator(targetConfig, fileConfig)
        JavaGeneratorUtils.writeToFile( cmakeGenerator.generateCode(cppSources), srcGenPath.resolve("CMakeLists.txt"))
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
        if (version != null) {
            val cmakeReturnCode = runCmake(context)

            if (cmakeReturnCode == 0 && runMake) {
                // If cmake succeeded, run make
                val makeCommand = createMakeCommand(fileConfig.buildPath, version)
                val makeReturnCode =
                    CppValidator(fileConfig, errorReporter, codeMaps).run(makeCommand, context.cancelIndicator)

                if (makeReturnCode == 0) {
                    println("SUCCESS (compiling generated C++ code)")
                    println("Generated source code is in ${fileConfig.srcGenPath}")
                    println("Compiled binary is in ${fileConfig.binPath}")
                } else {
                    // If errors occurred but none were reported, then the following message is the best we can do.
                    if (!generator.errorsOccurred()) errorReporter.reportError("make failed with error code $makeReturnCode")
                }
            } else {
                errorReporter.reportError("cmake failed with error code $cmakeReturnCode")
            }
        }
        return errorReporter.errorsOccurred
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
            errorReporter.reportError(
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
        val reactorCppPath = fileConfig.outPath.resolve("build").resolve("reactor-cpp")

        // run cmake
        val cmakeCommand = createCmakeCommand(fileConfig.buildPath, fileConfig.outPath, reactorCppPath)
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

    private fun createMakeCommand(buildPath: Path, version: String): LFCommand {
        val makeArgs: List<String>
        if (version.compareVersion("3.12.0") < 0) {
            errorReporter.reportWarning("CMAKE is older than version 3.12. Parallel building is not supported.")
            makeArgs =
                listOf("--build", ".", "--target", "install", "--config", targetConfig.cmakeBuildType?.toString() ?: "Release")
        } else {
            val cores = Runtime.getRuntime().availableProcessors()
            makeArgs = listOf(
                "--build",
                ".",
                "--target",
                "install",
                "--parallel",
                cores.toString(),
                "--config",
                targetConfig.cmakeBuildType?.toString() ?: "Release"
            )
        }

        return commandFactory.createCommand("cmake", makeArgs, buildPath)
    }

    private fun createCmakeCommand(buildPath: Path, outPath: Path, reactorCppPath: Path): LFCommand {
        val cmd = commandFactory.createCommand(
            "cmake", listOf(
                "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
                "-DREACTOR_CPP_BUILD_DIR=${reactorCppPath.toUnixString()}",
                "-DCMAKE_INSTALL_BINDIR=${outPath.relativize(fileConfig.binPath).toUnixString()}",
                fileConfig.srcGenPath.toUnixString()
            ),
            buildPath
        )

        // prepare cmake
        if (targetConfig.compiler != null) {
            cmd.setEnvironmentVariable("CXX", targetConfig.compiler)
        }
        return cmd
    }
}