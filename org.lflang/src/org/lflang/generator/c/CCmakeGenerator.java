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

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;

/**
 * A helper class that generates a CMakefile that can be used to compile the generated C code.
 * 
 * Adapted from @see org.lflang.generator.CppCmakeGenerator.kt
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 *
 */
class CCmakeGenerator {

    FileConfig fileConfig;
    TargetConfig targetConfig;
    
    /**
     * Create an instance of CCmakeGenerator.
     * 
     * @param targetConfig The TargetConfig instance to use.
     * @param fileConfig The FileConfig instance to use.
     */
    CCmakeGenerator(TargetConfig targetConfig, FileConfig fileConfig) {
        this.fileConfig = fileConfig;
        this.targetConfig = targetConfig;
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
     * @return The content of the CMakeLists.txt.
     */
    StringBuilder generateCMakeCode(
            List<String> sources, 
            String executableName, 
            ErrorReporter errorReporter,
            boolean CppMode,
            boolean hasMain) {
        StringBuilder cMakeCode = new StringBuilder();
        
        List<String> additionalSources = new ArrayList<String>();
        for (String file: targetConfig.compileAdditionalSources) {
            var relativePath = fileConfig.getSrcGenPath().relativize(
                fileConfig.getSrcGenPath().resolve(Paths.get(file)));
            additionalSources.add(FileConfig.toUnixString(relativePath));
        }
        
        cMakeCode.append("cmake_minimum_required(VERSION 3.13)\n");
        cMakeCode.append("project("+executableName+" LANGUAGES C)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Require C11\n");
        cMakeCode.append("set(CMAKE_C_STANDARD 11)\n");
        cMakeCode.append("set(CMAKE_C_STANDARD_REQUIRED ON)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Require C++17\n");
        cMakeCode.append("set(CMAKE_CXX_STANDARD 17)\n");
        cMakeCode.append("set(CMAKE_CXX_STANDARD_REQUIRED ON)\n");
        cMakeCode.append("\n");
        
        // Follow the 
        cMakeCode.append("set(DEFAULT_BUILD_TYPE " + targetConfig.cmakeBuildType + ")\n");
        cMakeCode.append("if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)\n");
        cMakeCode.append("    set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE} CACHE STRING \"Choose the type of build.\" FORCE)\n");
        cMakeCode.append("endif()\n");
        
        cMakeCode.append("set(CoreLib core)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Check which system we are running on to select the correct platform support\n");
        cMakeCode.append("# file and assign the file's path to LF_PLATFORM_FILE\n");
        cMakeCode.append("if(${CMAKE_SYSTEM_NAME} STREQUAL \"Linux\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_linux_support.c)\n");
        if (CppMode) {
            // Suppress warnings about const char*.
            cMakeCode.append("    set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -Wno-write-strings\")\n");
        }
        cMakeCode.append("elseif(${CMAKE_SYSTEM_NAME} STREQUAL \"Darwin\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_macos_support.c)\n");
        if (CppMode) {
            // Suppress warnings about const char*.
            cMakeCode.append("    set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -Wno-write-strings\")\n");
        }
        cMakeCode.append("elseif(${CMAKE_SYSTEM_NAME} STREQUAL \"Windows\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_windows_support.c)\n");
        cMakeCode.append("    set(CMAKE_SYSTEM_VERSION 10.0)\n");
        cMakeCode.append("    message(\"Using Windows SDK version ${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}\")\n");
        cMakeCode.append("else()\n");
        cMakeCode.append("    message(FATAL_ERROR \"Your platform is not supported!"+
                " The C target supports Linux, MacOS and Windows.\")\n");
        cMakeCode.append("endif()\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("include_directories(${CoreLib})\n");
        cMakeCode.append("include_directories(${CoreLib}/platform)\n");
        cMakeCode.append("include_directories(${CoreLib}/federated)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("set(LF_MAIN_TARGET "+executableName+")\n");
        if (hasMain) {
            cMakeCode.append("# Declare a new executable target and list all its sources\n");
            cMakeCode.append("add_executable( ${LF_MAIN_TARGET} "+String.join("\n", sources)+" ${LF_PLATFORM_FILE} "+
                               String.join("\n", additionalSources)+")\n");
        } else {
            cMakeCode.append("# Declare a new library target and list all its sources\n");
            cMakeCode.append("add_library( ${LF_MAIN_TARGET} "+String.join("\n", sources)+" ${LF_PLATFORM_FILE} "+
                               String.join("\n", additionalSources)+")\n");
        }
        cMakeCode.append("\n");

        if (targetConfig.threads != 0 || targetConfig.tracing != null) {
            // If threaded computation is requested, add a the threads option.
            cMakeCode.append("# Find threads and link to it\n");
            cMakeCode.append("find_package(Threads REQUIRED)\n");
            cMakeCode.append("target_link_libraries( ${LF_MAIN_TARGET} Threads::Threads)\n");
            cMakeCode.append("\n");
            
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            cMakeCode.append("# Set the number of workers to enable threading\n");
            cMakeCode.append("target_compile_definitions( ${LF_MAIN_TARGET} PUBLIC NUMBER_OF_WORKERS="+targetConfig.threads+")\n");
            cMakeCode.append("\n");
        }
        
        // Check if CppMode is enabled
        if (CppMode) {
            // First enable the CXX language
            cMakeCode.append("enable_language(CXX)\n");
            // FIXME: Instead of mixing a C compiler and a C++ compiler, we use a 
            // CMake flag to set the language of all .c files to C++.
            // Also convert any additional sources. This is a deprecated functionality 
            // in clang, but intermingling C compiled code and C++ compiled code seems 
            // to require a substantial overhaul of the C target code structure. Instead, 
            // we force the usage of a C++ compiler on everything for now.
            for (String source: additionalSources) {
                cMakeCode.append("set_source_files_properties( "+source+" PROPERTIES LANGUAGE CXX)\n");
            }
            cMakeCode.append("set_source_files_properties(${LF_PLATFORM_FILE} PROPERTIES LANGUAGE CXX)\n");
        }
        
        if (targetConfig.compiler != null && !targetConfig.compiler.isBlank()) {
            if (CppMode) {
                // Set the CXX compiler to what the user has requested.
                cMakeCode.append("set(CMAKE_CXX_COMPILER "+targetConfig.compiler+")\n");
            } else {
                cMakeCode.append("set(CMAKE_C_COMPILER "+targetConfig.compiler+")\n");
            }
        }
        
        // Set the compiler flags
        // We can detect a few common libraries and use the proper target_link_libraries to find them            
        for (String compilerFlag : targetConfig.compilerFlags) {
            switch(compilerFlag.trim()) {
                case "-lm":
                    cMakeCode.append("target_link_libraries( ${LF_MAIN_TARGET} m)\n");
                    break;
                case "-lprotobuf-c":
                    cMakeCode.append("include(FindPackageHandleStandardArgs)\n");
                    cMakeCode.append("FIND_PATH( PROTOBUF_INCLUDE_DIR protobuf-c/protobuf-c.h)\n");
                    cMakeCode.append("find_library(PROTOBUF_LIBRARY \n"+
                                     "NAMES libprotobuf-c.a libprotobuf-c.so libprotobuf-c.dylib protobuf-c.lib protobuf-c.dll\n"+
                                     ")\n");
                    cMakeCode.append("find_package_handle_standard_args(libprotobuf-c DEFAULT_MSG PROTOBUF_INCLUDE_DIR PROTOBUF_LIBRARY)\n");
                    cMakeCode.append("target_include_directories( ${LF_MAIN_TARGET} PUBLIC ${PROTOBUF_INCLUDE_DIR} )\n");
                    cMakeCode.append("target_link_libraries( ${LF_MAIN_TARGET} ${PROTOBUF_LIBRARY})\n");
                    break;
                case "-O2":
                    if (targetConfig.compiler.equals("gcc") || CppMode) {
                        // Workaround for the pre-added -O2 option in the CGenerator.
                        // This flag is specific to gcc/g++ and the clang compiler
                        cMakeCode.append("add_compile_options( -O2 )\n");
                        cMakeCode.append("add_link_options( -O2 )\n");
                        break;
                    }
                default:
                    errorReporter.reportWarning("Using the flags target property with cmake is dangerous.\n"+
                                                " Use cmake-include instead.");
                    cMakeCode.append("add_compile_options( "+compilerFlag+" )\n");
                    cMakeCode.append("add_link_options( "+compilerFlag+")\n");
            }
        }
        cMakeCode.append("\n");
        
        // Add the install option
        cMakeCode.append("install(TARGETS ${LF_MAIN_TARGET}\n");
        cMakeCode.append("        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})\n");
        cMakeCode.append("\n");
        
        // Add the include file
        for (String includeFile : targetConfig.cmakeIncludesWithoutPath) {
            cMakeCode.append("include(\""+includeFile+"\")\n");
        } 
        
        return cMakeCode;
    }
}
