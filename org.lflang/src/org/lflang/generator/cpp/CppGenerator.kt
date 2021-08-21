/* Generator for Cpp target. */

/*************
 * Copyright (c) 2019-2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.cpp

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.*
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.Action
import org.lflang.lf.VarRef
import org.lflang.scoping.LFGlobalScopeProvider
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

class CppGenerator(
    private val cppFileConfig: CppFileConfig,
    errorReporter: ErrorReporter,
    private val scopeProvider: LFGlobalScopeProvider
) :
    GeneratorBase(cppFileConfig, errorReporter) {

    companion object {
        /** Path to the Cpp lib directory (relative to class path)  */
        const val libDir = "/lib/Cpp"

        /** Default version of the reactor-cpp runtime to be used during compilation */
        const val defaultRuntimeVersion = "e404a8c7a48f5c8bcca25a82c3ae862ca0d23434"
    }

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (errorsOccurred()) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
            return
        }

        generateFiles(fsa)

        if (targetConfig.noCompile || errorsOccurred()) {
            println("Exiting before invoking target compiler.")
        } else {
            doCompile()
        }
    }

    private fun generateFiles(fsa: IFileSystemAccess2) {
        val srcGenPath = fileConfig.srcGenPath
        val relSrcGenPath = fileConfig.srcGenBasePath.relativize(srcGenPath)

        val mainReactor = mainDef.reactorClass.toDefinition()

        // copy static library files over to the src-gen directory
        val genIncludeDir = srcGenPath.resolve("__include__")
        copyFileFromClassPath("${libDir}/lfutil.hh", genIncludeDir.resolve("lfutil.hh").toString())
        copyFileFromClassPath("${libDir}/time_parser.hh", genIncludeDir.resolve("time_parser.hh").toString())
        copyFileFromClassPath("${libDir}/3rd-party/CLI11.hpp", genIncludeDir.resolve("CLI").resolve("CLI11.hpp").toString())

        // keep a list of all source files we generate
        val cppSources = mutableListOf<Path>()

        // generate the main source file (containing main())
        val mainGenerator = CppMainGenerator(mainReactor, targetConfig, cppFileConfig)
        val mainFile = Paths.get("main.cc")
        cppSources.add(mainFile)
        fsa.generateFile(relSrcGenPath.resolve(mainFile).toString(), mainGenerator.generateCode())

        // generate header and source files for all reactors
        for (r in reactors) {
            val generator = CppReactorGenerator(r, cppFileConfig, errorReporter)
            val headerFile = cppFileConfig.getReactorHeaderPath(r)
            val sourceFile = if (r.isGeneric) cppFileConfig.getReactorHeaderImplPath(r) else cppFileConfig.getReactorSourcePath(r)
            if (!r.isGeneric)
                cppSources.add(sourceFile)

            fsa.generateFile(relSrcGenPath.resolve(headerFile).toString(), generator.generateHeader())
            fsa.generateFile(relSrcGenPath.resolve(sourceFile).toString(), generator.generateSource())
        }

        // generate file level preambles for all resources
        for (r in resources) {
            val generator = CppPreambleGenerator(r, cppFileConfig, scopeProvider)
            val sourceFile = cppFileConfig.getPreambleSourcePath(r)
            val headerFile = cppFileConfig.getPreambleHeaderPath(r)
            cppSources.add(sourceFile)

            fsa.generateFile(relSrcGenPath.resolve(headerFile).toString(), generator.generateHeader())
            fsa.generateFile(relSrcGenPath.resolve(sourceFile).toString(), generator.generateSource())
        }

        // generate the cmake script
        val cmakeGenerator = CppCmakeGenerator(targetConfig, cppFileConfig)
        fsa.generateFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), cmakeGenerator.generateCode(cppSources))
    }

    fun doCompile() {
        val outPath = fileConfig.outPath

        val buildPath = outPath.resolve("build").resolve(topLevelName)
        val reactorCppPath = outPath.resolve("build").resolve("reactor-cpp")

        // make sure the build directory exists
        Files.createDirectories(buildPath)

        val cores = Runtime.getRuntime().availableProcessors()

        val makeCommand = commandFactory.createCommand(
            "cmake",
            listOf(
                "--build", ".", "--target", "install", "--parallel", cores.toString(), "--config",
                targetConfig.cmakeBuildType?.toString() ?: "Release"
            ),
            buildPath
        )

        val cmakeCommand = commandFactory.createCommand(
            "cmake", listOf(
                "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
                "-DREACTOR_CPP_BUILD_DIR=${reactorCppPath.toUnixString()}",
                "-DCMAKE_INSTALL_BINDIR=${outPath.relativize(fileConfig.binPath).toUnixString()}",
                fileConfig.srcGenPath.toUnixString()
            ),
            buildPath
        )
        if (makeCommand == null || cmakeCommand == null) {
            errorReporter.reportError(
                "The C++ target requires CMAKE >= 3.02 to compile the generated code. " +
                        "Auto-compiling can be disabled using the \"no-compile: true\" target property."
            )
            return
        }

        // prepare cmake
        if (targetConfig.compiler != null) {
            cmakeCommand.setEnvironmentVariable("CXX", targetConfig.compiler)
        }

        // run cmake
        val cmakeReturnCode = cmakeCommand.run()

        if (cmakeReturnCode == 0) {
            // If cmake succeeded, run make
            val makeReturnCode = makeCommand.run()

            if (makeReturnCode == 0) {
                println("SUCCESS (compiling generated C++ code)")
                println("Generated source code is in ${fileConfig.srcGenPath}")
                println("Compiled binary is in ${fileConfig.binPath}")
            } else {
                errorReporter.reportError("make failed with error code $makeReturnCode")
            }
        } else {
            errorReporter.reportError("cmake failed with error code $cmakeReturnCode")
        }
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action the action to schedule
     * @param port the port to read from
     */
    override fun generateDelayBody(action: Action, port: VarRef): String {
        // Since we cannot easily decide whether a given type evaluates
        // to void, we leave this job to the target compiler, by calling
        // the template function below.
        return """
        // delay body for ${action.name}
        lfutil::after_delay(&${action.name}, &${port.name});
        """.trimIndent()
    }

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port.
     * @param action the action that triggers the reaction
     * @param port the port to write to
     */
    override fun generateForwardBody(action: Action, port: VarRef): String {
        // Since we cannot easily decide whether a given type evaluates
        // to void, we leave this job to the target compiler, by calling
        // the template function below.
        return """
        // forward body for ${action.name}
        lfutil::after_forward(&${action.name}, &${port.name});
        """.trimIndent()
    }

    override fun generateDelayGeneric() = "T"

    override fun generateAfterDelaysWithVariableWidth() = false

    override fun supportsGenerics() = true

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetTagIntervalType() = targetUndefinedType

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = TODO()
    override fun getTargetVariableSizeListType(baseType: String) = TODO()

    override fun getTargetUndefinedType() = TODO()

    override fun getTarget() = Target.CPP
}
