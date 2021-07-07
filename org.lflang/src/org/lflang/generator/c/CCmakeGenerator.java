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

import java.io.File;
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
    
    CCmakeGenerator(TargetConfig targetConfig, FileConfig fileConfig) {
        this.fileConfig = fileConfig;
        this.targetConfig = targetConfig;
    }
    
    /**
     * Generate the contents of a CMakeLists.txt that builds the provided LF C 'sources'. Any error will be
     * reported in the 'errorReporter'.
     * 
     * @param sources A list of .c files to build.
     * @param executableName The name of the output executable
     * @param errorReporter Used to report errors.
     * @return The content of the CMakeLists.txt
     */
    StringBuilder generateCMakeCode(List<String> sources, String executableName, ErrorReporter errorReporter) {
        StringBuilder cMakeCode = new StringBuilder();
        
        // Resolve path to the cmake include file if one was provided
        String includeFile = targetConfig.cmakeInclude;
        if (!includeFile.isBlank()) {
            try {
                includeFile = FileConfig.toUnixString(fileConfig.srcPath.resolve(includeFile));
            } catch (Exception e) {
                errorReporter.reportError(e.getMessage());
            }
        }
        
        cMakeCode.append("cmake_minimum_required(VERSION 3.5)\n");
        cMakeCode.append("project("+executableName+"1.0.0 LANGUAGES C)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Require C11\n");
        cMakeCode.append("set(CMAKE_C_STANDARD 11)\n");
        cMakeCode.append("set(CMAKE_C_STANDARD_REQUIRED ON)\n");
        cMakeCode.append("set(CMAKE_C_EXTENSIONS OFF)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("set(CoreLib "+fileConfig.getSrcGenPath()+File.separator+"core)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Check which system we are running on to select the correct platform support\n");
        cMakeCode.append("# file and assign the file's path to LF_PLATFORM_FILE\n");
        cMakeCode.append("if(${CMAKE_SYSTEM_NAME} STREQUAL \"Linux\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_linux_support.c)\n");
        cMakeCode.append("elseif(${CMAKE_SYSTEM_NAME} STREQUAL \"Darwin\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_macos_support.c)\n");
        cMakeCode.append("elseif(${CMAKE_SYSTEM_NAME} STREQUAL \"Windows\")\n");
        cMakeCode.append("    set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_windows_support.c)\n");
        cMakeCode.append("else()\n");
        cMakeCode.append("    message(FATAL_ERROR \"Your platform is not supported!"+
                " The C target supports Linux, MacOS and Windows.\")\n");
        cMakeCode.append("endif()\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("include_directories(${CoreLib})\n");
        cMakeCode.append("include_directories(${CoreLib}/platform)\n");
        cMakeCode.append("include_directories(${CoreLib}/federated)\n");
        cMakeCode.append("\n");
        
        cMakeCode.append("# Declare a new executable target and list all its sources\n");
        cMakeCode.append("add_executable( "+executableName+" "+String.join("\n", sources)+" ${LF_PLATFORM_FILE})\n");
        cMakeCode.append("\n");

        if (targetConfig.threads != 0 || targetConfig.tracing != null) {
            // If threaded computation is requested, add a the threads option.
            cMakeCode.append("# Find threads and link to it\n");
            cMakeCode.append("find_package(Threads REQUIRED)\n");
            cMakeCode.append("target_link_libraries("+executableName+" Threads::Threads)");
            cMakeCode.append("\n");
            
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            cMakeCode.append("# Set the number of workers to enable threading\n");
            cMakeCode.append("target_compile_definitions("+executableName+" PUBLIC NUMBER_OF_WORKERS="+targetConfig.threads+")\n");
            cMakeCode.append("\n");
        }
        
        // Add the install option
        cMakeCode.append("install(TARGETS "+executableName+"\n");
        cMakeCode.append("        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})\n");
        
        if (!includeFile.isBlank()) {
            cMakeCode.append("include("+includeFile+")\n");
        }
        
        return cMakeCode;
    }
}
