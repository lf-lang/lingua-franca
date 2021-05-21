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

package org.lflang.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.*
import org.lflang.Target
import org.lflang.lf.Action
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef
import java.nio.file.Path

/* *******************************************************************************************
 * The following definitions are shortcuts to access static members of FileConfig and ASTUtils
 *
 * TODO these should likely be moved to a common place in the future
 */

val Resource.name: String get() = FileConfig.getName(this)

fun Path.toUnixString(): String = FileConfig.toUnixString(this)

val Reactor.isGeneric get() = ASTUtils.isGeneric(this.toDefinition())

/* ********************************************************************************************/

/** Prepend each line of the rhs multiline string with the lhs prefix.
 *
 * This is a neat little trick that allows for convenient insertion of multiline strings in string templates
 * while correctly managing the indentation. Consider this multiline string:
 * ```
 * val foo = """
 *    void foo() {
 *        // do something useful
 *    }""".trimIndent()
 * ```
 *
 * It could be inserted into a higher level multiline string like this:
 *
 * ```
 * val bar = """
 *     |class Bar {
 * ${" |    "..foo}
 *     |};""".trimMargin()
 * ```
 *
 * This will produce the expected output:
 * ```
 * class Bar {
 *     void foo() {
 *         // do something useful
 *     }
 * };
 * ```
 */
operator fun String.rangeTo(str: String) = str.prependIndent(this)

class CppGenerator : GeneratorBase() {

    /** Path to the Cpp lib directory (relative to class path)  */
    private val libDir = "/lib/Cpp"

    /** Relative path to the directory where all source files for this resource should be generated in. */
    private val Resource.genDir get() = fileConfig.getDirectory(this).resolve(this.name)

    /** Path to the preamble header file corresponding to this resource */
    private val Resource.preambleHeaderPath get() = this.genDir.resolve("_lf_preamble.hh")

    /** Path to the preamble source file corresponding to this resource */
    private val Resource.preambleSourcePath get() = this.genDir.resolve("_lf_preamble.cc")

    /** Path to the header file corresponding to this reactor */
    private val Reactor.headerPath get() = this.eResource().genDir.resolve("${this.name}.hh")

    /** Path to the implementation header file corresponding to this reactor (needed for generic reactors) */
    private val Reactor.headerImplPath get() = this.eResource().genDir.resolve("${this.name}_impl.hh")

    /** Path to the source file corresponding to this reactor (needed for non generic reactors)  */
    private val Reactor.sourcPath get() = this.eResource().genDir.resolve("${this.name}.cc")

    /** Convert a log level to a severity number understood by the reactor-cpp runtime. */
    private val TargetProperty.LogLevel.severity
        get() = when (this) {
            TargetProperty.LogLevel.ERROR -> 1
            TargetProperty.LogLevel.WARN -> 2
            TargetProperty.LogLevel.INFO -> 3
            TargetProperty.LogLevel.LOG -> 4
            TargetProperty.LogLevel.DEBUG -> 4
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
    }

    private fun generateFiles(fsa: IFileSystemAccess2) {
        val srcGenPath = fileConfig.srcGenPath
        val relSrcGenPath = fileConfig.srcGenBasePath.relativize(srcGenPath)

        // generate the cmake script
        fsa.generateFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), generateCmake())

        // copy static library files over to the src-gen directory
        val genIncludeDir = srcGenPath.resolve("__include__")
        copyFileFromClassPath("${libDir}/lfutil.hh", genIncludeDir.resolve("lfutil.hh").toString())
        copyFileFromClassPath("${libDir}/time_parser.hh", genIncludeDir.resolve("time_parser.hh").toString())
        copyFileFromClassPath("${libDir}/3rd-party/CLI11.hpp", genIncludeDir.resolve("CLI").resolve("CLI11.hpp").toString())
    }

    @Suppress("LocalVariableName") // allows us to use capital S as variable name below
    private fun generateCmake(): String {
        val runtimeVersion = targetConfig.runtimeVersion ?: "26e6e641916924eae2e83bbf40cbc9b933414310"

        // FIXME Is there a way to simplify this line?
        val includeFile = if (targetConfig.cmakeInclude.isNullOrBlank()) null else FileConfig.toUnixString(fileConfig.srcPath.resolve(targetConfig.cmakeInclude))

        // compile a list of all source files produced by this generator
        val reactorSourceFiles = reactors.filter { !it.isGeneric }.map { it.sourcPath.toUnixString() }
        val preambleSourceFiles = resources.map { it.preambleSourcePath.toUnixString() }
        val sourceFiles = reactorSourceFiles + preambleSourceFiles

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
            |set(DEFAULT_BUILD_TYPE ${targetConfig.cmakeBuildType}"})
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