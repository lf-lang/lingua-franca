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

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Mode;
import org.lflang.TargetConfig;
import org.lflang.util.LFCommand;


/**
 * Responsible for creating and executing the necessary CMake command to compile code that is generated
 * by the CGenerator. This class uses CMake to compile.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 */
class CCmakeCompiler extends CCompiler {
    // FIXME: This does not override the compileCCommand
    //  method because there is no single command that
    //  compiles; however, it is not clear why
    //  compileCCommand is public in the first place.

    /**
     * Create an instance of CCmakeCompiler.
     * 
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter Used to report errors.
     */
    public CCmakeCompiler(TargetConfig targetConfig, FileConfig fileConfig, ErrorReporter errorReporter) {
        super(targetConfig, fileConfig, errorReporter);
    }
    
    /** 
     * Run the C compiler by invoking cmake and make.
     * 
     * @param file The source file to compile without the .c extension.
     * @param noBinary If true, the compiler will create a .o output instead of a binary. 
     *  If false, the compile command will produce a binary.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    public boolean runCCompiler(String file, boolean noBinary) throws IOException {
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        // Make sure the build directory exists
        Files.createDirectories(buildPath);
        int configureReturnCode = configure();
        int buildReturnCode = 0;
        // Do not compile if the code generator is being
        //  invoked from an IDE or text editor without the
        //  user's explicit request.
        if (configureReturnCode == 0) {
            buildReturnCode = build();
        }
        return configureReturnCode == 0 && buildReturnCode == 0;
    }
    
    /**
     * Return a command to configure the specified C file
     * using CMake.
     * This produces a C-specific configure command.
     */
    private LFCommand configureCmakeCommand() {

        if (!targetConfig.compileLibraries.isEmpty()) {
            errorReporter.reportError("The current CMake build system does not support -l libraries.\n"+
                                        "Use the 'cmake-include' target property to include a CMakeLists file\n"+
                                        "with the appropriate library discovery syntax.");
        }
        
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        
        LFCommand command = commandFactory.createCommand(
                "cmake", List.of(
                        "-DCMAKE_INSTALL_PREFIX="+FileConfig.toUnixString(fileConfig.getOutPath()),
                        "-DCMAKE_INSTALL_BINDIR="+FileConfig.toUnixString(
                                fileConfig.getOutPath().relativize(
                                        fileConfig.binPath
                                        )
                                ),
                        FileConfig.toUnixString(fileConfig.getSrcGenPath())
                    ),
                buildPath);
        if (command == null) {
            errorReporter.reportError(
                    "The C target requires CMAKE >= 3.5 to compile the generated code. " +
                            "Auto-compiling can be disabled using the \"no-compile: true\" target property.");
        }
        return command;
    }
    
    /**
     * Return a command to build (compile and link) the
     * specified C file using CMake.
     * This produces a C-specific build command.
     *
     * It appears that configuration and build cannot
     * happen in one command. Therefore, this is separated
     * into a configuration command and a build command.
     */
    private LFCommand buildCmakeCommand() {
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        String cores = String.valueOf(Runtime.getRuntime().availableProcessors());
        LFCommand command =  commandFactory.createCommand(
                "cmake", List.of(
                        "--build", ".", "--target", "install", "--parallel", cores, "--config",
                        targetConfig.cmakeBuildType!=null ? targetConfig.cmakeBuildType.toString() : "Release"
                    ),
                buildPath);
        if (command == null) {
            errorReporter.reportError(
                    "The C target requires CMAKE >= 3.5 to compile the generated code. " +
                            "Auto-compiling can be disabled using the \"no-compile: true\" target property.");
        }
        return command;
    }

    private int configure() {
        LFCommand configure = configureCmakeCommand();
        if (configure == null) {
            return 1;
        }

        // Use the user-specified compiler if any
        if (targetConfig.compiler != null) {
            // cmakeEnv.remove("CXX");
            if (targetConfig.compiler.equals("g++") || targetConfig.compiler.equals("CC")) {
                // Interpret this as the user wanting their .c programs to be treated as
                // C++ files. We can't just simply use g++ to compile C code. We use a
                // specific CMake flag to set the language of all .c files to C++.
            } else {
                configure.replaceEnvironmentVariable("CC", targetConfig.compiler);
            }
            // cmakeEnv.put("CXX", targetConfig.compiler);
        }

        int returnCode = configure.run();

        if (returnCode != 0 && fileConfig.getCompilerMode() == Mode.STANDALONE) {
            // FIXME: Why is the error code attributed to the compiler if it is actually received via CMake? Is it
            //  possible that the source of the error code is not the compiler?
            errorReporter.reportError(
                targetConfig.compiler + " terminated with error code " + returnCode
                    + ". Cmake output:\n" + configure.getOutput().toString()
            );
        }

        return returnCode;
    }

    /**
     * Compiles the generated target code.
     * @return the return code from invoking CMake
     */
    private int build() {
        LFCommand build = buildCmakeCommand();

        int makeReturnCode = build.run();

        if (makeReturnCode != 0) {
            errorReporter.reportError(
                "CMake terminated with error code " + makeReturnCode + ". Output:\n" + build.getOutput().toString()
            );
        }

        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (build.getErrors().toString().length() > 0) {
            errorReporter.reportError(build.getErrors().toString());
        }
        return makeReturnCode;
    }
    
}
