package org.lflang.generator.cpp

import org.lflang.TargetProperty
import org.lflang.generator.CodeMap
import org.lflang.generator.LFGeneratorContext
import org.lflang.toUnixString
import org.lflang.util.FileUtil
import org.lflang.util.LFCommand
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

/** C++ platform generator for the default native platform  without additional dependencies.*/
class CppStandaloneGenerator(generator: CppGenerator) :
    CppPlatformGenerator(generator) {

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
        if (version != null) {
            val cmakeReturnCode = runCmake(context)

            if (cmakeReturnCode == 0 && runMake) {
                // If cmake succeeded, run make
                val makeCommand = createMakeCommand(fileConfig.buildPath, version, fileConfig.name)
                val makeReturnCode = CppValidator(fileConfig, errorReporter, codeMaps).run(makeCommand, context.cancelIndicator)
                var installReturnCode = 0
                if (makeReturnCode == 0) {
                    val installCommand = createMakeCommand(fileConfig.buildPath, version, "install")
                    installReturnCode = installCommand.run(context.cancelIndicator)
                    if (installReturnCode == 0) {
                        println("SUCCESS (compiling generated C++ code)")
                        println("Generated source code is in ${fileConfig.srcGenPath}")
                        println("Compiled binary is in ${fileConfig.binPath}")
                    }
                }
                if ((makeReturnCode != 0 || installReturnCode != 0) && !errorReporter.errorsOccurred) {
                    // If errors occurred but none were reported, then the following message is the best we can do.
                    errorReporter.reportError("make failed with error code $makeReturnCode")
                }
            }
            if (cmakeReturnCode != 0) {
                errorReporter.reportError("cmake failed with error code $cmakeReturnCode")
            }
        }
        return !errorReporter.errorsOccurred
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

    private fun buildTypeToCmakeConfig(type: TargetProperty.BuildType?) = when (type) {
        null                          -> "Release"
        TargetProperty.BuildType.TEST -> "Debug"
        else                          -> type.toString()
    }

    private fun createMakeCommand(buildPath: Path, version: String, target: String): LFCommand {
        val makeArgs: List<String>
        if (version.compareVersion("3.12.0") < 0) {
            errorReporter.reportWarning("CMAKE is older than version 3.12. Parallel building is not supported.")
            makeArgs =
                listOf("--build", ".", "--target", target, "--config", targetConfig.cmakeBuildType?.toString() ?: "Release")
        } else {
            val cores = Runtime.getRuntime().availableProcessors()
            makeArgs = listOf(
                "--build",
                ".",
                "--target",
                target,
                "--parallel",
                cores.toString(),
                "--config",
                buildTypeToCmakeConfig(targetConfig.cmakeBuildType)
            )
        }

        return commandFactory.createCommand("cmake", makeArgs, buildPath)
    }

    private fun createCmakeCommand(buildPath: Path, outPath: Path): LFCommand {
        val cmd = commandFactory.createCommand(
            "cmake", listOf(
                "-DCMAKE_BUILD_TYPE=${targetConfig.cmakeBuildType}",
                "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
                "-DCMAKE_INSTALL_BINDIR=${outPath.relativize(fileConfig.binPath).toUnixString()}",
                "-DREACTOR_CPP_VALIDATE=${if (targetConfig.noRuntimeValidation) "OFF" else "ON"}",
                "-DREACTOR_CPP_TRACE=${if (targetConfig.tracing != null) "ON" else "OFF"}",
                "-DREACTOR_CPP_LOG_LEVEL=${targetConfig.logLevel.severity}",
                "-DLF_SRC_PKG_PATH=${fileConfig.srcPkgPath}",
                fileConfig.srcGenBasePath.toUnixString()
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