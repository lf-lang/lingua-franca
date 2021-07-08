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

import java.io.ByteArrayOutputStream;
import java.nio.file.Path;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Mode;
import org.lflang.TargetConfig;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.GeneratorBase.ExecutionEnvironment;


/**
 * Responsible for creating and executing the necessary CMake command to compile code that is generated
 * by the CGenerator. This class uses CMake to compile.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 *
 * FIXME: This class has a strong coupling with GeneratorBase. However, that is because
 *  the GeneratorBase contains a lot of unrelated functions that need to be factored out
 *  into separate classes.
 */
class CCmakeCompiler extends CCompiler {

    /**
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param generator The generator that is using this compiler.
     */
    public CCmakeCompiler(
            TargetConfig targetConfig, 
            FileConfig fileConfig,
            GeneratorBase generator
    ) {
        super(targetConfig, fileConfig, generator);
    }
    
    /** 
     * Run the C compiler by invoking cmake and make.
     * 
     * @param file The source file to compile without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    @Override
    public 
    boolean runCCompiler(String file, boolean doNotLinkIfNoMain, ErrorReporter errorReporter) {
        ProcessBuilder compile = compileCmakeCommand(file, doNotLinkIfNoMain, errorReporter);
        if (compile == null) {
            return false;
        }
        
        // Set the build directory to be "build"
        Path buildPath = fileConfig.getSrcGenPath().resolve("build");
        // Make sure the build directory exists
        FileConfig.createDirectories(buildPath);
        compile.directory(buildPath.toFile());

        ByteArrayOutputStream stderr = new ByteArrayOutputStream();
        int cMakeReturnCode = generator.executeCommand(compile, stderr);
        
        if (cMakeReturnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
            errorReporter.reportError(targetConfig.compiler+" returns error code "+cMakeReturnCode);
        }
        
        int makeReturnCode = 0;

        if (cMakeReturnCode == 0) {            
            ProcessBuilder build = buildCmakeCommand(file, doNotLinkIfNoMain, errorReporter);
            
            build.directory(buildPath.toFile());
            
            makeReturnCode = generator.executeCommand(build, stderr);
            
            if (makeReturnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
                errorReporter.reportError(targetConfig.compiler+" returns error code "+makeReturnCode);
            }
            
        }
        
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (stderr.toString().length() > 0 && fileConfig.getCompilerMode() == Mode.INTEGRATED) {
            generator.reportCommandErrors(stderr.toString());
        }
        
        return ((cMakeReturnCode == 0) && (makeReturnCode == 0));
    }
    
    
    /**
     * Return a command to compile the specified C file using CMake.
     * This produces a C-specific compile command.
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * @param errorReporter Used to report errors to the user.
     */
    public ProcessBuilder compileCmakeCommand(
            String fileToCompile, 
            boolean doNotLinkIfNoMain, 
            ErrorReporter errorReporter
    ) {
        ExecutionEnvironment env = generator.findCommandEnv(
                targetConfig.compiler,
                "The C target requires CMAKE >= 3.5 to compile the generated code if 'cmake' is requested\n" +
                        " in the target property. Auto-compiling can be disabled using \n"+ 
                        "the \"no-compile: true\" target property.",
                true
        );
        

        if (!targetConfig.compileLibraries.isEmpty()) {
            errorReporter.reportError("The current CMake build system does not support -l libraries.\n"+
                                        "Use the 'cmake-include' target property to include a CMakeLists file\n"+
                                        "with the appropriate library discovery syntax.");
        }
        
        return generator.createCommand(
                "cmake", List.of(
                        "-DCMAKE_INSTALL_PREFIX="+FileConfig.toUnixString(fileConfig.getOutPath()),
                        "-DCMAKE_INSTALL_BINDIR="+FileConfig.toUnixString(
                                fileConfig.getOutPath().relativize(
                                        fileConfig.binPath
                                        )
                                ),
                        FileConfig.toUnixString(fileConfig.getSrcGenPath())
                    ),
                fileConfig.getOutPath(),
                env);
    }
    
    
    /**
     * Return a command to build the specified C file using CMake.
     * This produces a C-specific build command.
     * 
     * @note It appears that configuration and build cannot happen in one command.
     *  Therefore, this is separated into a compile and a build command. 
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * @param errorReporter Used to report errors to the user.
     */
    public ProcessBuilder buildCmakeCommand(
            String fileToCompile, 
            boolean doNotLinkIfNoMain, 
            ErrorReporter errorReporter
    ) {
        ExecutionEnvironment env = generator.findCommandEnv(
                targetConfig.compiler,
                "The C target requires CMAKE >= 3.5 to compile the generated code if 'cmake' is requested\n" +
                        " in the target property. Auto-compiling can be disabled using \n"+ 
                        "the \"no-compile: true\" target property.",
                true
        );
        String cores = String.valueOf(Runtime.getRuntime().availableProcessors());
        return generator.createCommand(
                "cmake", List.of(
                        "--build", ".", "--target", "install", "--parallel", cores, "--config",
                        ((targetConfig.cmakeBuildType!=null) ? toString() : "Release")
                    ),
                fileConfig.getOutPath(), 
                env);
    }
    
}
