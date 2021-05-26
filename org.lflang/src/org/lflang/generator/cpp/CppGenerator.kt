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
import org.lflang.FileConfig
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.Action
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef
import org.lflang.toDefinition

class CppGenerator : GeneratorBase() {

    companion object {
        /** Path to the Cpp lib directory (relative to class path)  */
        const val libDir = "/lib/Cpp"

        /** Default version of the reactor-cpp runtime to be used during compilation */
        const val defaultRuntimeVersion = "26e6e641916924eae2e83bbf40cbc9b933414310"
    }

    /** Get the file configuration object as `CppFileConfig` */
    private val cppFileConfig: CppFileConfig get() = super.fileConfig!! as CppFileConfig

    /**
     * Set the fileConfig field to point to the specified resource using the specified
     * file-system access and context.
     * @param resource The resource (Eclipse-speak for a file).
     * @param fsa The Xtext abstraction for the file system.
     * @param context The generator context (whatever that is).
     */
    override fun setFileConfig(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.fileConfig = CppFileConfig(resource, fsa, context)
        topLevelName = fileConfig.name
    }

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (generatorErrorsOccurred) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
            return
        }

        generateFiles(fsa)

        if (targetConfig.noCompile || generatorErrorsOccurred) {
            println("Exiting before invoking target compiler.")
        } else {
            doCompile()
        }
    }

    private fun generateFiles(fsa: IFileSystemAccess2) {
        val srcGenPath = fileConfig.srcGenPath
        val relSrcGenPath = fileConfig.srcGenBasePath.relativize(srcGenPath)

        val mainReactor = mainDef.reactorClass.toDefinition()

        // generate the main source file (containing main())
        fsa.generateFile(relSrcGenPath.resolve("main.cc").toString(), generateMain(mainReactor))

        // generate the cmake script
        fsa.generateFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), generateCmake())

        // copy static library files over to the src-gen directory
        val genIncludeDir = srcGenPath.resolve("__include__")
        copyFileFromClassPath("${libDir}/lfutil.hh", genIncludeDir.resolve("lfutil.hh").toString())
        copyFileFromClassPath("${libDir}/time_parser.hh", genIncludeDir.resolve("time_parser.hh").toString())
        copyFileFromClassPath("${libDir}/3rd-party/CLI11.hpp", genIncludeDir.resolve("CLI").resolve("CLI11.hpp").toString())

        // generate header and source files for all reactors
        for (r in reactors) {
            val generator = CppReactorGenerator(r, cppFileConfig)
            val headerFile = cppFileConfig.getReactorHeaderPath(r)
            val sourceFile = if (r.isGeneric) cppFileConfig.getReactorHeaderImplPath(r) else cppFileConfig.getReactorSourcePath(r)

            fsa.generateFile(relSrcGenPath.resolve(headerFile).toString(), generator.header())
            fsa.generateFile(relSrcGenPath.resolve(sourceFile).toString(), generator.source())
        }
    }

    private fun generateMain(main: Reactor) = """
    ${" |"..fileComment(main.eResource())}
        |
        |#include <chrono>
        |#include <thread>
        |#include <memory>
        |
        |#include "reactor-cpp/reactor-cpp.hh"
        |
        |using namespace std::chrono_literals;
        |using namespace reactor::operators;
        |
        |#include "time_parser.hh"
        |
        |#include "CLI/CLI11.hpp"
        |
        |#include "${cppFileConfig.getReactorHeaderPath(main).toUnixString()}"
        |
        |class Timeout : public reactor::Reactor {
        | private:
        |  reactor::Timer timer;
        |
        |  reactor::Reaction r_timer{"r_timer", 1, this, [this]() { environment()->sync_shutdown(); }};
        |
        |
        | public:
        |  Timeout(const std ::string& name, reactor::Environment* env, reactor::Duration timeout)
        |    : reactor::Reactor(name, env)
        |    , timer{ "timer", this, reactor::Duration::zero(), timeout } {}
        |
        |  void assemble () override { r_timer.declare_trigger(& timer); }
        |};
        |
        |int main(int argc, char **argv) {
        |  CLI::App app ("$topLevelName Reactor Program");
        |
        |  unsigned threads = ${if (targetConfig.threads != 0) targetConfig.threads else "std::thread::hardware_concurrency()"};
        |  app.add_option("-t,--threads", threads, "the number of worker threads used by the scheduler", true);
        |
        |  reactor::Duration timeout = ${targetConfig.timeout?.toCode() ?: "reactor::Duration::zero()"}
        |  auto opt_timeout = app . add_option ("-o,--timeout", timeout, "Time after which the execution is aborted.");
        |
        |  opt_timeout->check([](const std::string& val ){ return validate_time_string(val); });
        |  opt_timeout->type_name("'FLOAT UNIT'");
        |  opt_timeout->default_str(time_to_quoted_string(timeout));
        |
        |  bool fast{${targetConfig.fastMode}};
        |  app.add_flag("-f,--fast", fast, "Allow logical time to run faster than physical time.");
        |
        |  bool keepalive {${targetConfig.keepalive}};
        |  app.add_flag("-k,--keepalive", keepalive, "Continue execution even when there are no events to process.");
        |
        |  /* TODO!
        |  «FOR p : mainReactor.parameters»
        |
        |    «p.targetType» «p.name» = «p.targetInitializer»;
        |    auto opt_«p.name» = app.add_option("--«p.name»", «p.name», "The «p.name» parameter passed to the main reactor «mainReactor.name».");
        |    «IF p.inferredType.isTime»
        |          opt_«p.name»->check([](const std::string& val){ return validate_time_string(val); });
        |          opt_«p.name»->type_name("'FLOAT UNIT'");
        |          opt_«p.name»->default_str(time_to_quoted_string(«p.name»));
        |    «ENDIF»
        |  «ENDFOR»
        |  */
        |
        |  app.get_formatter()->column_width(50);
        |
        |  CLI11_PARSE(app, argc, argv);
        |
        |  reactor::Environment e{threads, keepalive, fast};
        |
        |  // instantiate the main reactor
        |  auto main = std ::make_unique<${main.name}> ("${main.name}")
        |  // TODO support parameters: , &e«FOR p : mainReactor.parameters BEFORE ", " SEPARATOR ", "»«p.name»«ENDFOR»);
        |
        |  // optionally instantiate the timeout reactor
        |  std::unique_ptr<Timeout> t{nullptr};
        |  if (timeout != reactor::Duration::zero()) {
        |    t = std::make_unique<Timeout>("Timeout", & e, timeout);
        |  }
        |
        |  // execute the reactor program
        |  e.assemble();
        |  auto thread = e . startup ();
        |  thread.join();
        |
        |  return 0;
        |}
        """.trimMargin()

    private fun generateCmake(): String {
        val runtimeVersion = targetConfig.runtimeVersion ?: defaultRuntimeVersion

        // Resolve path to the cmake include file if one was provided
        val includeFile = targetConfig.cmakeInclude
            ?.takeIf { it.isNotBlank() }
            ?.let { fileConfig.srcPath.resolve(it).toUnixString() }

        // compile a list of all source files produced by this generator
        val reactorSourceFiles = reactors.filter { !it.isGeneric }.map { cppFileConfig.getReactorSourcePath(it).toUnixString() }
        val preambleSourceFiles = resources.map { cppFileConfig.getPreambleSourcePath(it).toUnixString() }
        val sourceFiles = reactorSourceFiles + preambleSourceFiles

        @Suppress("LocalVariableName") // allows us to use capital S as variable name below
        val S = '$' // a little trick to escape the dollar sign with $S

        return """
            |cmake_minimum_required(VERSION 3.5)
            |project(${topLevelName} VERSION 1.0.0 LANGUAGES CXX)
            |
            |# require C++ 17
            |set(CMAKE_CXX_STANDARD 17)
            |set(CMAKE_CXX_STANDARD_REQUIRED ON)
            |set(CMAKE_CXX_EXTENSIONS OFF)
            |
            |include($S{CMAKE_ROOT}/Modules/ExternalProject.cmake)
            |include(GNUInstallDirs)
            |
            |set(DEFAULT_BUILD_TYPE "${targetConfig.cmakeBuildType}")
            |if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
            |set    (CMAKE_BUILD_TYPE "$S{DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
            |endif()
            |
        ${
            if (targetConfig.externalRuntimePath != null) """
                |find_package(reactor-cpp PATHS ${targetConfig.externalRuntimePath}")
            """.trimIndent()
            else """
                |if(NOT REACTOR_CPP_BUILD_DIR)
                |    set(REACTOR_CPP_BUILD_DIR "" CACHE STRING "Choose the directory to build reactor-cpp in." FORCE)
                |endif()
                |
                |ExternalProject_Add(dep-reactor-cpp
                |   PREFIX "$S{REACTOR_CPP_BUILD_DIR}"
                |   GIT_REPOSITORY "https://github.com/tud-ccc/reactor-cpp.git"
                |   GIT_TAG "$runtimeVersion"
                |   CMAKE_ARGS
                |   -DCMAKE_BUILD_TYPE:STRING=$S{CMAKE_BUILD_TYPE}
                |   -DCMAKE_INSTALL_PREFIX:PATH=$S{CMAKE_INSTALL_PREFIX}
                |   -DCMAKE_INSTALL_BINDIR:PATH=$S{CMAKE_INSTALL_BINDIR}
                |   -DCMAKE_CXX_COMPILER=$S{CMAKE_CXX_COMPILER}
                |   -DREACTOR_CPP_VALIDATE=${if (targetConfig.noRuntimeValidation) "OFF" else "ON"}
                |   -DREACTOR_CPP_TRACE=${if (targetConfig.tracing != null) "ON" else "OFF"} 
                |   -DREACTOR_CPP_LOG_LEVEL=${targetConfig.logLevel.severity}
                |)
                |
                |set(REACTOR_CPP_LIB_DIR "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_LIBDIR}")
                |set(REACTOR_CPP_BIN_DIR "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_BINDIR}")
                |set(REACTOR_CPP_LIB_NAME "$S{CMAKE_SHARED_LIBRARY_PREFIX}reactor-cpp$S{CMAKE_SHARED_LIBRARY_SUFFIX}")
                |set(REACTOR_CPP_IMPLIB_NAME "$S{CMAKE_STATIC_LIBRARY_PREFIX}reactor-cpp$S{CMAKE_STATIC_LIBRARY_SUFFIX}")
                |
                |add_library(reactor-cpp SHARED IMPORTED)
                |add_dependencies(reactor-cpp dep-reactor-cpp)
                |if(WIN32)
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_IMPLIB "$S{REACTOR_CPP_LIB_DIR}/$S{REACTOR_CPP_IMPLIB_NAME}")
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "$S{REACTOR_CPP_BIN_DIR}/$S{REACTOR_CPP_LIB_NAME}")
                |else()
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "$S{REACTOR_CPP_LIB_DIR}/$S{REACTOR_CPP_LIB_NAME}")
                |endif()
                |
                |if (APPLE)
                |   file(RELATIVE_PATH REL_LIB_PATH "$S{REACTOR_CPP_BIN_DIR}" "$S{REACTOR_CPP_LIB_DIR}")
                |   set(CMAKE_INSTALL_RPATH "@executable_path/$S{REL_LIB_PATH}")
                |else ()
                |   set(CMAKE_INSTALL_RPATH "$S{REACTOR_CPP_LIB_DIR}")
                |endif ()
            """.trimIndent()
        }
            |
            |set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
            |
            |set(LF_MAIN_TARGET ${topLevelName})
            |
            |add_executable($S{LF_MAIN_TARGET}
            |    main.cc
        ${" |    "..sourceFiles.joinToString("\n")}
            |)
            |target_include_directories($S{LF_MAIN_TARGET} PUBLIC
            |    "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_INCLUDEDIR}"
            |    "$S{PROJECT_SOURCE_DIR}"
            |    "$S{PROJECT_SOURCE_DIR}/__include__"
            |)
            |target_link_libraries($S{LF_MAIN_TARGET} reactor-cpp)
            |
            |install(TARGETS $S{LF_MAIN_TARGET})
        ${
            if (includeFile == null) "" else """
                |
                |include($includeFile)""".trimIndent()
        }
            """.trimMargin()
    }

    fun doCompile() {
        val outPath = fileConfig.outPath

        val buildPath = outPath.resolve("build").resolve(topLevelName)
        val reactorCppPath = outPath.resolve("build").resolve("reactor-cpp")

        // make sure the build directory exists
        FileConfig.createDirectories(buildPath)

        val makeBuilder = createCommand(
            "cmake",
            listOf(
                "--build", ".", "--target", "install", "--config",
                targetConfig.cmakeBuildType?.toString() ?: "Release"
            ),
            outPath, // FIXME: it doesn't make sense to provide a search path here, createCommand() should accept null
            "The C++ target requires CMAKE >= 3.02 to compile the generated code. " +
                    "Auto-compiling can be disabled using the \"no-compile: true\" target property.",
            true
        )
        val cmakeBuilder = createCommand(
            "cmake", listOf(
                "-DCMAKE_INSTALL_PREFIX=${outPath.toUnixString()}",
                "-DREACTOR_CPP_BUILD_DIR=${reactorCppPath.toUnixString()}",
                "-DCMAKE_INSTALL_BINDIR=${outPath.relativize(fileConfig.binPath).toUnixString()}",
                fileConfig.srcGenPath.toUnixString()
            ),
            outPath, // FIXME: it doesn't make sense to provide a search path here, createCommand() should accept null
            "The C++ target requires CMAKE >= 3.02 to compile the generated code" +
                    "Auto-compiling can be disabled using the \"no-compile: true\" target property.",
            true
        )
        if (makeBuilder == null || cmakeBuilder == null) {
            return
        }

        // prepare cmake
        cmakeBuilder.directory(buildPath.toFile())
        if (targetConfig.compiler != null) {
            val cmakeEnv = cmakeBuilder.environment()
            cmakeEnv["CXX"] = targetConfig.compiler
        }

        // run cmake
        val cmakeReturnCode = executeCommand(cmakeBuilder)

        if (cmakeReturnCode == 0) {
            // If cmake succeeded, prepare and run make
            makeBuilder.directory(buildPath.toFile())
            val makeReturnCode = executeCommand(makeBuilder)

            if (makeReturnCode == 0) {
                println("SUCCESS (compiling generated C++ code)")
                println("Generated source code is in ${fileConfig.srcGenPath}")
                println("Compiled binary is in ${fileConfig.binPath}")
            } else {
                reportError("make failed with error code $makeReturnCode")
            }
        } else {
            reportError("cmake failed with error code $cmakeReturnCode")
        }
    }

    override fun generateDelayBody(action: Action, port: VarRef) = TODO()
    override fun generateForwardBody(action: Action, port: VarRef) = TODO()
    override fun generateDelayGeneric() = TODO()
    override fun supportsGenerics() = true

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetTagIntervalType() = targetUndefinedType

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = TODO()
    override fun getTargetVariableSizeListType(baseType: String) = TODO()

    override fun getTargetUndefinedType() = TODO()

    override fun getTarget() = Target.CPP
}
