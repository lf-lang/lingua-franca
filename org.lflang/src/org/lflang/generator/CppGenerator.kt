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
import org.lflang.FileConfig
import org.lflang.Target
import org.lflang.generator.GeneratorBase
import org.lflang.lf.Action
import org.lflang.lf.VarRef

class CppGenerator : GeneratorBase() {

    /** Path to the Cpp lib directory (relative to class path)  */
    val libDir = "/lib/Cpp"

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGeneratre() in GeneratorBase
        if (generatorErrorsOccurred) return;

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
        }

        generateFiles(fsa)
    }

    private fun generateFiles(fsa: IFileSystemAccess2) {
        val srcGenPath = this.fileConfig.getSrcGenPath();
        val relSrcGenPath = this.fileConfig.srcGenBasePath.relativize(srcGenPath);

        // generate the cmake script
        fsa.generateFile(relSrcGenPath.resolve("CMakeLists.txt").toString(), generateCmake())

        // copy static library files over to the src-gen directory
        val genIncludeDir = srcGenPath.resolve("__include__")
        copyFileFromClassPath("${libDir}/lfutil.hh", genIncludeDir.resolve("lfutil.hh").toString())
        copyFileFromClassPath("${libDir}/time_parser.hh", genIncludeDir.resolve("time_parser.hh").toString())
        copyFileFromClassPath("${libDir}/3rd-party/CLI11.hpp", genIncludeDir.resolve("CLI").resolve("CLI11.hpp").toString())
    }

    private fun generateCmake() = """
        |cmake_minimum_required(VERSION 3.5)
        |project(${topLevelName} VERSION 1.0.0 LANGUAGES CXX)

        |# require C++ 17
        |set(CMAKE_CXX_STANDARD 17)
        |set(CMAKE_CXX_STANDARD_REQUIRED ON)
        |set(CMAKE_CXX_EXTENSIONS OFF)

        |include(${'$'}{CMAKE_ROOT}/Modules/ExternalProject.cmake)
        |include(GNUInstallDirs)

        |set(DEFAULT_BUILD_TYPE ${if (targetConfig.cmakeBuildType == null) "Release" else "${targetConfig.cmakeBuildType}"})
        |if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        |    set(CMAKE_BUILD_TYPE "${'$'}{DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
        |endif()
        ${if (targetConfig.externalRuntimePath != null)
            "|find_package(reactor-cpp PATHS ${targetConfig.externalRuntimePath}" 
        else
            """
            |if(NOT REACTOR_CPP_BUILD_DIR)
            |    set(REACTOR_CPP_BUILD_DIR "" CACHE STRING "Choose the directory to build reactor-cpp in." FORCE)
            |endif()

            |ExternalProject_Add(dep-reactor-cpp
            |   PREFIX "${'$'}{REACTOR_CPP_BUILD_DIR}"
            |   GIT_REPOSITORY "https://github.com/tud-ccc/reactor-cpp.git"
            |   GIT_TAG "${if (targetConfig.runtimeVersion != null) targetConfig.runtimeVersion else "26e6e641916924eae2e83bbf40cbc9b933414310"}"
            |   CMAKE_ARGS
            |   -DCMAKE_BUILD_TYPE:STRING=${'$'}{CMAKE_BUILD_TYPE}
            |   -DCMAKE_INSTALL_PREFIX:PATH=${'$'}{CMAKE_INSTALL_PREFIX}
            |   -DCMAKE_INSTALL_BINDIR:PATH=${'$'}{CMAKE_INSTALL_BINDIR}
            |   -DCMAKE_CXX_COMPILER=${'$'}{CMAKE_CXX_COMPILER}
            |   -DREACTOR_CPP_VALIDATE=${if (targetConfig.noRuntimeValidation) "OFF" else "ON"}
            |   -DREACTOR_CPP_TRACE=${if (targetConfig.tracing != null) "ON" else "OFF"} 
            |   # TODO «IF targetConfig.logLevel !== null»-DREACTOR_CPP_LOG_LEVEL=«logLevelsToInts.get(targetConfig.logLevel)»«ELSE»«logLevelsToInts.get(LogLevel.INFO)»«ENDIF»
            |)

            |set(REACTOR_CPP_LIB_DIR "${'$'}{CMAKE_INSTALL_PREFIX}/${'$'}{CMAKE_INSTALL_LIBDIR}")
            |set(REACTOR_CPP_BIN_DIR "${'$'}{CMAKE_INSTALL_PREFIX}/${'$'}{CMAKE_INSTALL_BINDIR}")
            |set(REACTOR_CPP_LIB_NAME "${'$'}{CMAKE_SHARED_LIBRARY_PREFIX}reactor-cpp${'$'}{CMAKE_SHARED_LIBRARY_SUFFIX}")
            |set(REACTOR_CPP_IMPLIB_NAME "${'$'}{CMAKE_STATIC_LIBRARY_PREFIX}reactor-cpp${'$'}{CMAKE_STATIC_LIBRARY_SUFFIX}")

            |add_library(reactor-cpp SHARED IMPORTED)
            |add_dependencies(reactor-cpp dep-reactor-cpp)
            |if(WIN32)
            |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_IMPLIB "${'$'}{REACTOR_CPP_LIB_DIR}/${'$'}{REACTOR_CPP_IMPLIB_NAME}")
            |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "${'$'}{REACTOR_CPP_BIN_DIR}/${'$'}{REACTOR_CPP_LIB_NAME}")
            |else()
            |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "${'$'}{REACTOR_CPP_LIB_DIR}/${'$'}{REACTOR_CPP_LIB_NAME}")
            |endif()

            |if (APPLE)
            |   file(RELATIVE_PATH REL_LIB_PATH "${'$'}{REACTOR_CPP_BIN_DIR}" "${'$'}{REACTOR_CPP_LIB_DIR}")
            |   set(CMAKE_INSTALL_RPATH "@executable_path/${'$'}{REL_LIB_PATH}")
            |else ()
            |   set(CMAKE_INSTALL_RPATH "${'$'}{REACTOR_CPP_LIB_DIR}")
            |endif ()
            """
        }

        |set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

        |set(LF_MAIN_TARGET ${topLevelName})

        |add_executable(${'$'}{LF_MAIN_TARGET}
        |    main.cc
        |#TODO
        |#«FOR r : reactors»
        |#«IF !r.toDefinition.isGeneric»«r.toDefinition.sourceFile.toUnixString»«ENDIF»
        |#«ENDFOR»
        |#«FOR r : resources»
        |#«r.preambleSourceFile.toUnixString»
        |#«ENDFOR»
        |)
        |target_include_directories(${'$'}{LF_MAIN_TARGET} PUBLIC
        |    "${'$'}{CMAKE_INSTALL_PREFIX}/${'$'}{CMAKE_INSTALL_INCLUDEDIR}"
        |    "${'$'}{PROJECT_SOURCE_DIR}"
        |    "${'$'}{PROJECT_SOURCE_DIR}/__include__"
    )
    |target_link_libraries(${'$'}{LF_MAIN_TARGET} reactor-cpp)

    |install(TARGETS ${'$'}{LF_MAIN_TARGET})

    |${if (targetConfig.cmakeInclude.isNullOrEmpty()) "" else FileConfig.toUnixString(fileConfig.srcPath.resolve(targetConfig.cmakeInclude))}
    """.trimMargin()

    override fun generateDelayBody(action: Action, port: VarRef) = null // TODO
    override fun generateForwardBody(action: Action, port: VarRef) = null // TODO
    override fun generateDelayGeneric() = null // TODO
    override fun supportsGenerics() = true // TODO

    override fun getTargetTimeType() = "reactor::Duration"
    override fun getTargetTagType() = "reactor::Tag"

    override fun getTargetTagIntervalType() = getTargetUndefinedType()

    override fun getTargetFixedSizeListType(baseType: String, size: Int) = null // TODO
    override fun getTargetVariableSizeListType(baseType: String) = null // TODO

    override fun getTargetUndefinedType() = null // TODO

    override fun getTarget() = Target.CPP
}