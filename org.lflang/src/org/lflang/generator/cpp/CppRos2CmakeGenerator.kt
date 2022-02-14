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

import org.lflang.TargetProperty
import org.lflang.generator.PrependOperator
import org.lflang.toUnixString
import java.nio.file.Path

/** Code generator for producing a cmake script for compiling all generating C++ sources */
class CppRos2CmakeGenerator(generator: CppGenerator) {
    private val targetConfig = generator.targetConfig
    private val fileConfig = generator.cppFileConfig

    companion object {
        const val compilerIdName: String = "CXX_COMPILER_ID"
    }

    fun generateCode(sources: List<Path>): String {
        @Suppress("LocalVariableName") // allows us to use capital S as variable name below
        val S = '$' // a little trick to escape the dollar sign with $S

        return with(PrependOperator) {
            """
                |cmake_minimum_required(VERSION 3.5)
                |project(${fileConfig.name} VERSION 0.0.0 LANGUAGES CXX)
                |
                |# require C++ 17
                |set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard is cached for visibility in external tools." FORCE)
                |set(CMAKE_CXX_STANDARD_REQUIRED ON)
                |set(CMAKE_CXX_EXTENSIONS OFF)
                |
                |set(DEFAULT_BUILD_TYPE "${targetConfig.cmakeBuildType}")
                |if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
                |set    (CMAKE_BUILD_TYPE "$S{DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
                |endif()
                |
                
                |# Invoke find_package() for all build and buildtool dependencies.
                |find_package(ament_cmake_auto REQUIRED)
                |ament_auto_find_build_dependencies()
                |
                |set(LF_MAIN_TARGET ${fileConfig.name})
                |
                |ament_auto_add_library($S{LF_MAIN_TARGET} SHARED
            ${" |    "..sources.joinToString("\n") { "src/$it" }}
                |)
                |ament_target_dependencies($S{LF_MAIN_TARGET} rclcpp std_msgs reactor-cpp)
                |target_include_directories($S{LF_MAIN_TARGET} PUBLIC
                |    "$S{CMAKE_INSTALL_PREFIX}/$S{CMAKE_INSTALL_INCLUDEDIR}"
                |    "$S{PROJECT_SOURCE_DIR}"
                |    "$S{PROJECT_SOURCE_DIR}/__include__"
                |)
                |
                |rclcpp_components_register_node($S{LF_MAIN_TARGET}
                |  PLUGIN "$S{LF_MAIN_TARGET}"
                |  EXECUTABLE $S{LF_MAIN_TARGET}_exe
                |)
                |
                |if(MSVC)
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE /W4)
                |else()
                |  target_compile_options($S{LF_MAIN_TARGET} PRIVATE -Wall -Wextra -pedantic)
                |endif()
                |
                |ament_auto_package()
            """.trimMargin()
        }
    }
}
