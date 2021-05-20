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
import org.lflang.lf.Action
import org.lflang.lf.VarRef

class CppGenerator : GeneratorBase() {

    /** Path to the Cpp lib directory (relative to class path)  */
    private val libDir = "/lib/Cpp"

    override fun doGenerate(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) {
        super.doGenerate(resource, fsa, context)

        // stop if there are any errors found in the program by doGeneratre() in GeneratorBase
        if (generatorErrorsOccurred) return

        // abort if there is no main reactor
        if (mainDef == null) {
            println("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
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

    private fun generateCmake(): String {
        val runtimeVersion =
                if (targetConfig.runtimeVersion != null) targetConfig.runtimeVersion else "26e6e641916924eae2e83bbf40cbc9b933414310"

        val S = '$' // a little trick to escape the dollar sign with $S

        val sb = StringBuilder()
        sb.append("""
            |cmake_minimum_required(VERSION 3.5)
            |project(${topLevelName} VERSION 1.0.0 LANGUAGES CXX)

            |# require C++ 17
            |set(CMAKE_CXX_STANDARD 17)
            |set(CMAKE_CXX_STANDARD_REQUIRED ON)
            |set(CMAKE_CXX_EXTENSIONS OFF)

            |include($S{CMAKE_ROOT}/Modules/ExternalProject.cmake)
            |include(GNUInstallDirs)

            |set(DEFAULT_BUILD_TYPE ${targetConfig.cmakeBuildType}"})
            |if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
            |set    (CMAKE_BUILD_TYPE "$S{DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
            |endif()""")
        if (targetConfig.externalRuntimePath != null) {
            sb.append("|find_package(reactor-cpp PATHS ${targetConfig.externalRuntimePath}")
        } else {
            sb.append("""
                |if(NOT REACTOR_CPP_BUILD_DIR)
                |    set(REACTOR_CPP_BUILD_DIR "" CACHE STRING "Choose the directory to build reactor-cpp in." FORCE)
                |endif()

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
                |   # TODO «IF targetConfig.logLevel !== null»-DREACTOR_CPP_LOG_LEVEL=«logLevelsToInts.get(targetConfig.logLevel)»«ELSE»«logLevelsToInts.get(LogLevel.INFO)»«ENDIF»
                |)

                |set(REACTOR_CPP_LIB_DIR "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_LIBDIR}")
                |set(REACTOR_CPP_BIN_DIR "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_BINDIR}")
                |set(REACTOR_CPP_LIB_NAME "$S{CMAKE_SHARED_LIBRARY_PREFIX}reactor-cpp$S{CMAKE_SHARED_LIBRARY_SUFFIX}")
                |set(REACTOR_CPP_IMPLIB_NAME "$S{CMAKE_STATIC_LIBRARY_PREFIX}reactor-cpp$S{CMAKE_STATIC_LIBRARY_SUFFIX}")

                |add_library(reactor-cpp SHARED IMPORTED)
                |add_dependencies(reactor-cpp dep-reactor-cpp)
                |if(WIN32)
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_IMPLIB "$S{REACTOR_CPP_LIB_DIR}/$S{REACTOR_CPP_IMPLIB_NAME}")
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "$S{REACTOR_CPP_BIN_DIR}/$S{REACTOR_CPP_LIB_NAME}")
                |else()
                |   set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "$S{REACTOR_CPP_LIB_DIR}/$S{REACTOR_CPP_LIB_NAME}")
                |endif()

                |if (APPLE)
                |   file(RELATIVE_PATH REL_LIB_PATH "$S{REACTOR_CPP_BIN_DIR}" "$S{REACTOR_CPP_LIB_DIR}")
                |   set(CMAKE_INSTALL_RPATH "@executable_path/$S{REL_LIB_PATH}")
                |else ()
                |   set(CMAKE_INSTALL_RPATH "$S{REACTOR_CPP_LIB_DIR}")
                |endif ()""")
        }
        sb.append("""
            |set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

            |set(LF_MAIN_TARGET ${topLevelName})

            |add_executable($S{LF_MAIN_TARGET}
            |    main.cc
            |#TODO
            |#«FOR r : reactors»
            |#«IF !r.toDefinition.isGeneric»«r.toDefinition.sourceFile.toUnixString»«ENDIF»
            |#«ENDFOR»
            |#«FOR r : resources»
            |#«r.preambleSourceFile.toUnixString»
            |#«ENDFOR»
            |)
            |target_include_directories($S{LF_MAIN_TARGET} PUBLIC
            |    "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_INCLUDEDIR}"
            |    "$S{PROJECT_SOURCE_DIR}"
            |    "$S{PROJECT_SOURCE_DIR}/__include__"
            |)
            |target_link_libraries($S{LF_MAIN_TARGET} reactor-cpp)

            |install(TARGETS $S{LF_MAIN_TARGET})""")

        if (!targetConfig.cmakeInclude.isNullOrEmpty()) {
            val includeFile = FileConfig.toUnixString(fileConfig.srcPath.resolve(targetConfig.cmakeInclude))
            sb.append("""

            |include($includeFile)
            """)
        }

        return sb.toString().trimMargin()
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