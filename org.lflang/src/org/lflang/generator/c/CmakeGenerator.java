/*************
 * Copyright (c) 2019-2021, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.c;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.Platform;
import org.lflang.generator.CodeBuilder;
import org.lflang.util.FileUtil;

/**
 * A helper class that generates a CMakefile that can be used to compile the generated C code.
 *
 * Adapted from @see org.lflang.generator.CppCmakeGenerator.kt
 *
 * @author Soroush Bateni <soroush@utdallas.edu>
 *
 */
public class CmakeGenerator {
    private static final String DEFAULT_INSTALL_CODE = """
        install(
            TARGETS ${LF_MAIN_TARGET}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    """;

    private final FileConfig fileConfig;
    private final List<String> additionalSources;
    private final SetUpMainTarget setUpMainTarget;
    private final String installCode;

    public CmakeGenerator(
        FileConfig fileConfig,
        List<String> additionalSources
    ) {
        this.fileConfig = fileConfig;
        this.additionalSources = additionalSources;
        this.setUpMainTarget = CmakeGenerator::setUpMainTarget;
        this.installCode = DEFAULT_INSTALL_CODE;
    }

    public CmakeGenerator(
        FileConfig fileConfig,
        List<String> additionalSources,
        SetUpMainTarget setUpMainTarget,
        String installCode
    ) {
        this.fileConfig = fileConfig;
        this.additionalSources = additionalSources;
        this.setUpMainTarget = setUpMainTarget;
        this.installCode = installCode;
    }

    /**
     * Generate the contents of a CMakeLists.txt that builds the provided LF C 'sources'. Any error will be
     * reported in the 'errorReporter'.
     *
     * @param sources A list of .c files to build.
     * @param executableName The name of the output executable.
     * @param errorReporter Used to report errors.
     * @param CppMode Indicate if the compilation should happen in C++ mode
     * @param hasMain Indicate if the .lf file has a main reactor or not. If not,
     *  a library target will be created instead of an executable.
     * @param cMakeExtras CMake-specific code that should be appended to the CMakeLists.txt.
     * @param targetConfig The TargetConfig instance to use.
     * @return The content of the CMakeLists.txt.
     */
    CodeBuilder generateCMakeCode(
        List<String> sources,
        String executableName,
        ErrorReporter errorReporter,
        boolean CppMode,
        boolean hasMain,
        String cMakeExtras,
        TargetConfig targetConfig
    ) {
        CodeBuilder cMakeCode = new CodeBuilder();

        List<String> additionalSources = new ArrayList<>();
        for (String file: targetConfig.compileAdditionalSources) {
            var relativePath = fileConfig.getSrcGenPath().relativize(
                fileConfig.getSrcGenPath().resolve(Paths.get(file)));
            additionalSources.add(FileUtil.toUnixString(relativePath));
        }
        additionalSources.addAll(this.additionalSources);
        cMakeCode.newLine();

        cMakeCode.pr("cmake_minimum_required(VERSION 3.13)");
        cMakeCode.pr("project("+executableName+" LANGUAGES C)");

        cMakeCode.newLine();
        cMakeCode.pr("# Require C11");
        cMakeCode.pr("set(CMAKE_C_STANDARD 11)");
        cMakeCode.pr("set(CMAKE_C_STANDARD_REQUIRED ON)");
        cMakeCode.newLine();

        cMakeCode.pr("# Require C++17");
        cMakeCode.pr("set(CMAKE_CXX_STANDARD 17)");
        cMakeCode.pr("set(CMAKE_CXX_STANDARD_REQUIRED ON)");
        cMakeCode.newLine();
        if (!targetConfig.cmakeIncludes.isEmpty()) {
            // The user might be using the non-keyword form of
            // target_link_libraries. Ideally we would detect whether they are
            // doing that, but it is easier to just always have a deprecation
            // warning.
            cMakeCode.pr("""
                cmake_policy(SET CMP0023 OLD)  # This causes deprecation warnings

                """
            );
        }

        // Set the build type
        cMakeCode.pr("set(DEFAULT_BUILD_TYPE " + targetConfig.cmakeBuildType + ")\n");
        cMakeCode.pr("if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)\n");
        cMakeCode.pr("    set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE} CACHE STRING \"Choose the type of build.\" FORCE)\n");
        cMakeCode.pr("endif()\n");
        cMakeCode.newLine();

        if (CppMode) {
            // Suppress warnings about const char*.
            cMakeCode.pr("set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -Wno-write-strings\")");
            cMakeCode.newLine();
        }

        if (targetConfig.platformOptions.platform != Platform.AUTO) {
            cMakeCode.pr("set(CMAKE_SYSTEM_NAME "+targetConfig.platformOptions.platform.getcMakeName()+")");
        }

        cMakeCode.pr(setUpMainTarget.getCmakeCode(
            hasMain,
            executableName,
            Stream.concat(additionalSources.stream(), sources.stream())
        ));

        cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE core)");

        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/)");
        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/api)");
        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core)");
        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/platform)");
        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/modal_models)");
        cMakeCode.pr("target_include_directories(${LF_MAIN_TARGET} PUBLIC include/core/utils)");

        if (targetConfig.threading || targetConfig.tracing != null) {
            // If threaded computation is requested, add the threads option.
            cMakeCode.pr("# Find threads and link to it");
            cMakeCode.pr("find_package(Threads REQUIRED)");
            cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE Threads::Threads)");
            cMakeCode.newLine();

            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            cMakeCode.pr("# Set the number of workers to enable threading");
            cMakeCode.pr("target_compile_definitions(${LF_MAIN_TARGET} PUBLIC NUMBER_OF_WORKERS="+targetConfig.workers+")");
            cMakeCode.newLine();
        }

        cMakeCode.pr("# Target definitions\n");
        targetConfig.compileDefinitions.forEach((key, value) -> cMakeCode.pr(
            "target_compile_definitions(${LF_MAIN_TARGET} PUBLIC "+key+"="+value+")\n"
        ));
        cMakeCode.newLine();

        if (CppMode) cMakeCode.pr("enable_language(CXX)");

        if (targetConfig.compiler != null && !targetConfig.compiler.isBlank()) {
            if (CppMode) {
                // Set the CXX compiler to what the user has requested.
                cMakeCode.pr("set(CMAKE_CXX_COMPILER "+targetConfig.compiler+")");
            } else {
                cMakeCode.pr("set(CMAKE_C_COMPILER "+targetConfig.compiler+")");
            }
            cMakeCode.newLine();
        }

        // Set the compiler flags
        // We can detect a few common libraries and use the proper target_link_libraries to find them
        for (String compilerFlag : targetConfig.compilerFlags) {
            switch(compilerFlag.trim()) {
                case "-lm":
                    cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE m)");
                    break;
                case "-lprotobuf-c":
                    cMakeCode.pr("include(FindPackageHandleStandardArgs)");
                    cMakeCode.pr("FIND_PATH( PROTOBUF_INCLUDE_DIR protobuf-c/protobuf-c.h)");
                    cMakeCode.pr("""
                         find_library(PROTOBUF_LIBRARY\s
                         NAMES libprotobuf-c.a libprotobuf-c.so libprotobuf-c.dylib protobuf-c.lib protobuf-c.dll
                         )""");
                    cMakeCode.pr("find_package_handle_standard_args(libprotobuf-c DEFAULT_MSG PROTOBUF_INCLUDE_DIR PROTOBUF_LIBRARY)");
                    cMakeCode.pr("target_include_directories( ${LF_MAIN_TARGET} PUBLIC ${PROTOBUF_INCLUDE_DIR} )");
                    cMakeCode.pr("target_link_libraries(${LF_MAIN_TARGET} PRIVATE ${PROTOBUF_LIBRARY})");
                    break;
                case "-O2":
                    if (Objects.equals(targetConfig.compiler, "gcc") || CppMode) {
                        // Workaround for the pre-added -O2 option in the CGenerator.
                        // This flag is specific to gcc/g++ and the clang compiler
                        cMakeCode.pr("add_compile_options(-O2)");
                        cMakeCode.pr("add_link_options(-O2)");
                        break;
                    }
                default:
                    errorReporter.reportWarning("Using the flags target property with cmake is dangerous.\n"+
                                                " Use cmake-include instead.");
                    cMakeCode.pr("add_compile_options( "+compilerFlag+" )");
                    cMakeCode.pr("add_link_options( "+compilerFlag+")");
            }
        }
        cMakeCode.newLine();

        // Add the install option
        cMakeCode.pr(installCode);
        cMakeCode.newLine();

        if (this.targetConfig.platformOptions.platform == Platform.ARDUINO) {
            cMakeCode.pr("target_link_arduino_libraries ( ${LF_MAIN_TARGET} AUTO_PUBLIC)");
            cMakeCode.pr("target_enable_arduino_upload(${LF_MAIN_TARGET})");
        }

        // Add the include file
        for (String includeFile : targetConfig.cmakeIncludesWithoutPath) {
            cMakeCode.pr("include(\""+includeFile+"\")");
        }
        cMakeCode.newLine();

        cMakeCode.pr(cMakeExtras);
        cMakeCode.newLine();

        return cMakeCode;
    }

    public interface SetUpMainTarget {
        String getCmakeCode(boolean hasMain, String executableName, Stream<String> cSources);
    }

    private static String setUpMainTarget(
        boolean hasMain,
        String executableName,
        Stream<String> cSources
    ) {
        var code = new CodeBuilder();
        code.pr("add_subdirectory(core)");
        code.newLine();
        code.pr("set(LF_MAIN_TARGET "+executableName+")");
        code.newLine();

        if (hasMain) {
            code.pr("# Declare a new executable target and list all its sources");
            code.pr("add_executable(");
        } else {
            code.pr("# Declare a new library target and list all its sources");
            code.pr("add_library(");
        }
        code.indent();
        code.pr("${LF_MAIN_TARGET}");
        cSources.forEach(code::pr);
        code.unindent();
        code.pr(")");
        code.newLine();
        return code.toString();
    }
}
