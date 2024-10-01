
package org.lflang.generator.uc

import org.lflang.FileConfig
import org.lflang.target.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.joinWithLn
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.PlatformProperty
import org.lflang.target.property.CmakeIncludeProperty
import org.lflang.target.property.ExternalRuntimePathProperty
import org.lflang.target.property.RuntimeVersionProperty
import org.lflang.toUnixString
import java.nio.file.Path

class UcCmakeGenerator(private val targetConfig: TargetConfig, private val fileConfig: FileConfig) {
    private val S = '$' // a little trick to escape the dollar sign with $S
    // FIXME: I literally don't know how to uppercase a string in Kotlin...
    private val platformName = targetConfig.get(PlatformProperty.INSTANCE).platform
    fun generateCmake(sources: List<Path>) = with(PrependOperator) {
        """
            |cmake_minimum_required(VERSION 3.5)
            |set(PLATFORM "FLEXPRET" CACHE STRING "Platform to target")
            |
            |if ($S{PLATFORM} STREQUAL "FLEXPRET")
            |   # Must include toolchain file before project(), otherwise cmake
            |   # will determine its own compiler settings
            |   include(${S}ENV{FP_SDK_PATH}/cmake/riscv-toolchain.cmake)
            |endif()
            |
            |project(${fileConfig.name} VERSION 0.0.0 LANGUAGES C)
            |
            |set(LF_MAIN_TARGET ${fileConfig.name})
            |set(SOURCES
        ${" |    "..sources.joinWithLn { it.toUnixString() }}
            |)
            |if ($S{PLATFORM} STREQUAL "POSIX")
            |   add_library($S{LF_MAIN_TARGET} STATIC $S{SOURCES})
            |elseif ($S{PLATFORM} STREQUAL "ZEPHYR")
            |   zephyr_library_named($S{LF_MAIN_TARGET})
            |   zephyr_library_sources($S{SOURCES})
            |   zephyr_library_link_libraries(kernel)
            |elseif ($S{PLATFORM} STREQUAL "FLEXPRET")
            |   add_library($S{LF_MAIN_TARGET} STATIC $S{SOURCES})
            |   # Include FlexPRET's SDK functionality
            |   include(${S}ENV{FP_SDK_PATH}/cmake/fp-app.cmake)
            |   add_subdirectory(${S}ENV{FP_SDK_PATH} BINARY_DIR)
            |else()
            |   message(FATAL_ERROR "PLATFORM not defined")
            |endif()
            |
            |add_subdirectory(reactor-uc)
            |
            |target_include_directories($S{LF_MAIN_TARGET} PUBLIC $S{CMAKE_CURRENT_SOURCE_DIR})
            |target_link_libraries($S{LF_MAIN_TARGET} PUBLIC reactor-uc)
            |
        """.trimMargin()
    }
}
