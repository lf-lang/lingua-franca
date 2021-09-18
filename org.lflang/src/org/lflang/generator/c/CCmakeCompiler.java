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
import org.lflang.generator.GeneratorBase;
import org.lflang.util.LFCommand;


/**
 * Responsible for creating and executing the necessary CMake command to compile code that is generated
 * by the CGenerator. This class uses CMake to compile.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 */
public class CCmakeCompiler extends CCompiler {
    
    /**
     * Create an instance of CCmakeCompiler.
     * 
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter Used to report errors.
     * @param CppMode Indicate if the compilation should happen in C++ mode
     */
    public CCmakeCompiler(
            TargetConfig targetConfig, 
            FileConfig fileConfig, 
            ErrorReporter errorReporter, 
            boolean CppMode
            ) {
        super(targetConfig, fileConfig, errorReporter, CppMode);
    }

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
     * @param generator An instance of GenratorBase, only used to report error line numbers
     *  in the Eclipse IDE.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    public boolean runCCompiler(String file, boolean noBinary, GeneratorBase generator) throws IOException {
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        // Make sure the build directory exists
        Files.createDirectories(buildPath);

        LFCommand compile = compileCmakeCommand(file, noBinary);
        if (compile == null) {
            return false;
        }

        // Use the user-specified compiler if any
        if (targetConfig.compiler != null) {
            if (CppMode) {
                // Set the CXX environment variable to change the C++ compiler.
                compile.replaceEnvironmentVariable("CXX", targetConfig.compiler);
            } else {
                // Set the CC environment variable to change the C compiler.
                compile.replaceEnvironmentVariable("CC", targetConfig.compiler);
            }
        }
        
        int cMakeReturnCode = compile.run();
        
        if (cMakeReturnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
            errorReporter.reportError(targetConfig.compiler+" returns error code "+cMakeReturnCode);
            
            // Check if the error thrown is due to the wrong compiler
            if (compile.getErrors().toString().contains("The CMAKE_C_COMPILER is set to a C++ compiler")) {
                // If so, print an appropriate error message
                if (targetConfig.compiler != null) {
                    errorReporter.reportError(
                            "A C++ compiler was requested in the compiler target property."
                                    + " Use the CCpp or the Cpp target instead.");
                } else {
                    errorReporter.reportError("\"A C++ compiler was detected."
                           + " The C target works best with a C compiler." 
                           + " Use the CCpp or the Cpp target instead.\"");
                }
            }
        }
        
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (compile.getErrors().toString().length() > 0 && fileConfig.getCompilerMode() == Mode.INTEGRATED) {
            generator.reportCommandErrors(compile.getErrors().toString());
        }
        
        int makeReturnCode = 0;

        if (cMakeReturnCode == 0) {            
            LFCommand build = buildCmakeCommand(file, noBinary);
            
            makeReturnCode = build.run();
            
            if (makeReturnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
                errorReporter.reportError(targetConfig.compiler+" returns error code "+makeReturnCode);
            }
            
            // For warnings (vs. errors), the return code is 0.
            // But we still want to mark the IDE.
            if (build.getErrors().toString().length() > 0 && fileConfig.getCompilerMode() == Mode.INTEGRATED) {
                generator.reportCommandErrors(build.getErrors().toString());
            }
            
        }
        
        return ((cMakeReturnCode == 0) && (makeReturnCode == 0));
    }
    
    
    /**
     * Return a command to compile the specified C file using CMake.
     * This produces a C-specific compile command.
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param noBinary If true, the compiler will create a .o output instead of a binary. 
     *  If false, the compile command will produce a binary.
     */
    public LFCommand compileCmakeCommand(
            String fileToCompile, 
            boolean noBinary
    ) {
        

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
                    "The C/CCpp target requires CMAKE >= 3.5 to compile the generated code. " +
                            "Auto-compiling can be disabled using the \"no-compile: true\" target property.");
        }
        return command;
    }
    
    
    /**
     * Return a command to build the specified C file using CMake.
     * This produces a C-specific build command.
     * 
     * @note It appears that configuration and build cannot happen in one command.
     *  Therefore, this is separated into a compile and a build command. 
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param noBinary If true, the compiler will create a .o output instead of a binary. 
     *  If false, the compile command will produce a binary.
     */
    public LFCommand buildCmakeCommand(
            String fileToCompile, 
            boolean noBinary
    ) { 
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        String cores = String.valueOf(Runtime.getRuntime().availableProcessors());
        LFCommand command =  commandFactory.createCommand(
                "cmake", List.of(
                        "--build", ".", "--target", "install", "--parallel", cores, "--config",
                        ((targetConfig.cmakeBuildType!=null) ? targetConfig.cmakeBuildType.toString() : "Release")
                    ),
                buildPath);
        if (command == null) {
            errorReporter.reportError(
                    "The C/CCpp target requires CMAKE >= 3.5 to compile the generated code. " +
                            "Auto-compiling can be disabled using the \"no-compile: true\" target property.");
        }
        return command;
    }
    
}
