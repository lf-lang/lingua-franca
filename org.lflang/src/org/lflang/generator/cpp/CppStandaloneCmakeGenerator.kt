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
import org.lflang.generator.PrependOperator
import org.lflang.joinWithLn
import org.lflang.toUnixString
import java.nio.file.Path

/** Code generator for producing a cmake script for compiling all generated C++ sources */
class CppStandaloneCmakeGenerator(private val targetConfig: TargetConfig, private val fileConfig: CppFileConfig) {

    companion object {
        /** Return the name of the variable that gives the includes of the given target. */
        fun includesVarName(buildTargetName: String): String = "TARGET_INCLUDE_DIRECTORIES_$buildTargetName"
        const val compilerIdName: String = "CXX_COMPILER_ID"
    }

    @Suppress("PrivatePropertyName") // allows us to use capital S as variable name below
    private val S = '$' // a little trick to escape the dollar sign with $S

    fun generateRootCmake(projectName: String): String {
        return """
            |cmake_minimum_required(VERSION 3.5)
            |project($projectName VERSION 0.0.0 LANGUAGES CXX)
            |
            |# The Test build type is the Debug type plus coverage generation
            |if(CMAKE_BUILD_TYPE STREQUAL "Test")
            |  set(CMAKE_BUILD_TYPE "Debug")
            |
            |  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            |    find_program(LCOV_BIN lcov)
            |    if(LCOV_BIN MATCHES "lcov$S")
            |      set(CMAKE_CXX_FLAGS "$S{CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
            |    else()
            |      message("Not producing code coverage information since lcov was not found")
            |    endif()
            |  else()
            |    message("Not producing code coverage information since the selected compiler is no gcc")
            |  endif()
            |endif()
            |
            |# require C++ 17
            |set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard is cached for visibility in external tools." FORCE)
            |set(CMAKE_CXX_STANDARD_REQUIRED ON)
            |set(CMAKE_CXX_EXTENSIONS OFF)
            |
            |# don't automatically build and install all targets
            |set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
            |
            |include($S{CMAKE_ROOT}/Modules/ExternalProject.cmake)
            |include(GNUInstallDirs)
            |
            |set(DEFAULT_BUILD_TYPE "${targetConfig.cmakeBuildType}")
            |if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
            |set    (CMAKE_BUILD_TYPE "$S{DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
            |endif()
            |
            |if (APPLE)
            |   file(RELATIVE_PATH REL_LIB_PATH 
            |        "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_BINDIR}"
            |        "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_LIBDIR}"
            |   )
            |   set(CMAKE_INSTALL_RPATH "@executable_path/$S{REL_LIB_PATH}")
            |else ()
            |   set(CMAKE_INSTALL_RPATH "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_LIBDIR}")
            |endif ()
            |
            |set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
            |
            |file(GLOB subdirs RELATIVE "$S{PROJECT_SOURCE_DIR}" "$S{PROJECT_SOURCE_DIR}/*")
            |foreach(subdir $S{subdirs})
            |  if(IS_DIRECTORY "$S{PROJECT_SOURCE_DIR}/$S{subdir}")
            |    if($S{subdir} MATCHES "reactor-cpp-.*")
            |      string(SUBSTRING $S{subdir} 12 -1 LF_REACTOR_CPP_SUFFIX)
            |      add_subdirectory("$S{subdir}")
            |    endif()
            |  endif()
            |endforeach()
            |foreach(subdir $S{subdirs})
            |  if(IS_DIRECTORY "$S{PROJECT_SOURCE_DIR}/$S{subdir}")
            |    if(EXISTS "$S{PROJECT_SOURCE_DIR}/$S{subdir}/.lf-cpp-marker")
            |      if(NOT $S{subdir} MATCHES "reactor-cpp-.*")
            |        add_subdirectory("$S{subdir}")
            |      endif()
            |    endif()
            |  endif()
            |endforeach()
        """.trimMargin()
    }

    fun generateSubdirCmake(): String {
        return """
            |file(GLOB subdirs RELATIVE "$S{CMAKE_CURRENT_SOURCE_DIR}" "$S{CMAKE_CURRENT_SOURCE_DIR}/*")
            |foreach(subdir $S{subdirs})
            |  if(IS_DIRECTORY "$S{CMAKE_CURRENT_SOURCE_DIR}/$S{subdir}")
            |    if(EXISTS "$S{CMAKE_CURRENT_SOURCE_DIR}/$S{subdir}/.lf-cpp-marker")
            |      add_subdirectory("$S{subdir}")
            |    endif()
            |  endif()
            |endforeach()
        """.trimMargin()
    }

    fun generateCmake(sources: List<Path>): String {
        // Resolve path to the cmake include files if any was provided
        val includeFiles = targetConfig.cmakeIncludes?.map { fileConfig.srcPath.resolve(it).toUnixString() }

        val reactorCppTarget = when {
            targetConfig.externalRuntimePath != null -> "reactor-cpp"
            targetConfig.runtimeVersion != null -> "reactor-cpp-${targetConfig.runtimeVersion}"
            else -> "reactor-cpp-default"
        }

        return with(PrependOperator) {
            """
                |cmake_minimum_required(VERSION 3.5)
                |project(${fileConfig.name} VERSION 0.0.0 LANGUAGES CXX)
                |
                |${if (targetConfig.externalRuntimePath != null) "find_package(reactor-cpp PATHS ${targetConfig.externalRuntimePath})" else ""}
                |
                |set(LF_MAIN_TARGET ${fileConfig.name})
                |
                |add_executable($S{LF_MAIN_TARGET}
            ${" |    "..sources.joinWithLn { it.toUnixString() }}
                |)
                |target_include_directories($S{LF_MAIN_TARGET} PUBLIC
                |    "$S{LF_SRC_PKG_PATH}/src"
                |    "$S{PROJECT_SOURCE_DIR}"
                |    "$S{PROJECT_SOURCE_DIR}/__include__"
                |)
                |target_link_libraries($S{LF_MAIN_TARGET} $reactorCppTarget)
                |
                |if(MSVC)
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE /W4)
                |else()
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE -Wall -Wextra -pedantic)
                |endif()
                |
                |install(TARGETS $S{LF_MAIN_TARGET}
                |        RUNTIME DESTINATION $S{CMAKE_INSTALL_BINDIR}
                |        OPTIONAL
                |)
                |
                |# Cache a list of the include directories for use with tools external to CMake and Make.
                |# This will only work if the subdirectory that sets up the library target has already been visited.
                |get_target_property(TARGET_INCLUDE_DIRECTORIES $S{LF_MAIN_TARGET} INCLUDE_DIRECTORIES)
                |get_target_property(REACTOR_CPP_INCLUDE_DIRECTORIES $reactorCppTarget INCLUDE_DIRECTORIES)
                |list(APPEND TARGET_INCLUDE_DIRECTORIES $S{REACTOR_CPP_INCLUDE_DIRECTORIES})
                |set(${includesVarName(fileConfig.name)} $S{TARGET_INCLUDE_DIRECTORIES} CACHE STRING "Directories included in the main target." FORCE)
                |set($compilerIdName $S{CMAKE_CXX_COMPILER_ID} CACHE STRING "The name of the C++ compiler." FORCE)
                |
            ${" |"..(includeFiles?.joinWithLn { "include(\"$it\")" } ?: "")}
            """.trimMargin()
        }
    }
}
