/*************
 * Copyright (c) 2021, TU Dresden.

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

import org.lflang.TargetConfig
import org.lflang.TargetProperty
import org.lflang.generator.PrependOperator
import org.lflang.toUnixString
import java.nio.file.Path

/** Code generator for producing a cmake script for compiling all generating C++ sources */
class CppCmakeGenerator(private val targetConfig: TargetConfig, private val fileConfig: CppFileConfig) {

    /** Convert a log level to a severity number understood by the reactor-cpp runtime. */
    private val TargetProperty.LogLevel.severity
        get() = when (this) {
            TargetProperty.LogLevel.ERROR -> 1
            TargetProperty.LogLevel.WARN  -> 2
            TargetProperty.LogLevel.INFO  -> 3
            TargetProperty.LogLevel.LOG   -> 4
            TargetProperty.LogLevel.DEBUG -> 4
        }

    fun generateCode(sources: List<Path>): String {
        val runtimeVersion = targetConfig.runtimeVersion ?: CppGenerator.defaultRuntimeVersion

        // Resolve path to the cmake include file if one was provided
        val includeFile = targetConfig.cmakeInclude
            ?.takeIf { it.isNotBlank() }
            ?.let { fileConfig.srcPath.resolve(it).toUnixString() }

        @Suppress("LocalVariableName") // allows us to use capital S as variable name below
        val S = '$' // a little trick to escape the dollar sign with $S

        return with(PrependOperator) {
            """
                |cmake_minimum_required(VERSION 3.5)
                |project(${fileConfig.name} VERSION 1.0.0 LANGUAGES CXX)
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
                    |find_package(reactor-cpp PATHS "${targetConfig.externalRuntimePath}")
                """.trimIndent() else """
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
                |set(LF_MAIN_TARGET ${fileConfig.name})
                |
                |add_executable($S{LF_MAIN_TARGET}
            ${" |    "..sources.joinToString("\n") { it.toUnixString() }}
                |)
                |target_include_directories($S{LF_MAIN_TARGET} PUBLIC
                |    "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_INCLUDEDIR}"
                |    "$S{PROJECT_SOURCE_DIR}"
                |    "$S{PROJECT_SOURCE_DIR}/__include__"
                |)
                |target_link_libraries($S{LF_MAIN_TARGET} reactor-cpp)
                |
                |if(MSVC)
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE /W4)
                |else()
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE -Wall -Wextra -pedantic)
                |endif()
                |
                |install(TARGETS $S{LF_MAIN_TARGET}
                |        RUNTIME DESTINATION $S{CMAKE_INSTALL_BINDIR}
                |)
            ${
                if (includeFile == null) "" else """
                    |
                    |include($includeFile)
                """.trimIndent()
            }
            """.trimMargin()
        }
    }
}