
package org.lflang.generator.uc

import org.lflang.FileConfig
import org.lflang.target.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.joinWithLn
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.CmakeIncludeProperty
import org.lflang.target.property.ExternalRuntimePathProperty
import org.lflang.target.property.RuntimeVersionProperty
import org.lflang.toUnixString
import java.nio.file.Path

class UcCmakeGenerator(private val targetConfig: TargetConfig, private val fileConfig: FileConfig) {
    private val S = '$' // a little trick to escape the dollar sign with $S
    fun generateCmake(sources: List<Path>) = with(PrependOperator) {
        """
            |cmake_minimum_required(VERSION 3.5)
            |project(${fileConfig.name} VERSION 0.0.0 LANGUAGES C)
            |set(PLATFORM "POSIX" CACHE STRING "Platform to target")
            |
            |set(LF_MAIN_TARGET ${fileConfig.name})
            |set(SOURCES
        ${" |    "..sources.joinWithLn { it.toUnixString() }}
            |)
            |# If we are targeting POSIX, which is only meant for testing purposes,
            |# then we directlu create an executable. If we are targeting Zephyr,
            |# we build a library that can be included into a Zephyr project.
            |if (PLATFORM STREQUAL "POSIX")
            |   add_executable($S{LF_MAIN_TARGET} $S{SOURCES})
            |elseif (PLATFORM STREQUAL "ZEPHYR")
            |   zephyr_library_named($S{LF_MAIN_TARGET})
            |   zephyr_library_sources($S{SOURCES})
            |   zephyr_library_link_libraries(kernel)
            |else()
            |   message(FATAL_ERROR "PLATFORM not defined")
            |endif()
            |
            |add_subdirectory(../reactor-uc reactor-uc)
            |
            |target_include_directories($S{LF_MAIN_TARGET} PUBLIC $S{CMAKE_CURRENT_SOURCE_DIR})
            |target_link_libraries($S{LF_MAIN_TARGET} PUBLIC reactor-uc)
            |
        """.trimMargin()
    }
}
