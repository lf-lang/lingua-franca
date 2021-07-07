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
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Mode;
import org.lflang.TargetConfig;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.GeneratorBase.ExecutionEnvironment;

/**
 * Responsible for creating and executing the necessary command to compile code that is generated
 * by the CGenerator.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 *
 * FIXME: This class has a strong coupling with GeneratorBase. However, that is because
 *  the GeneratorBase contains a lot of unrelated functions that need to be factored out
 *  into separate classes.
 */
public class CCompiler {

    FileConfig fileConfig;
    TargetConfig targetConfig;
    GeneratorBase generator;
    
    public CCompiler(TargetConfig targetConfig, FileConfig fileConfig, GeneratorBase generator) {
        this.fileConfig = fileConfig;
        this.targetConfig = targetConfig;
        this.generator = generator;
    }

    /** 
     * Run the C compiler.
     * 
     * @param file The source file to compile without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    public boolean runCCompiler(String file, boolean doNotLinkIfNoMain, ErrorReporter errorReporter) {
        ProcessBuilder compile = compileCCommand(file, doNotLinkIfNoMain, errorReporter);
        if (compile == null) {
            return false;
        }

        ByteArrayOutputStream stderr = new ByteArrayOutputStream();
        int returnCode = generator.executeCommand(compile, stderr);

        if (returnCode != 0 && fileConfig.getCompilerMode() != Mode.INTEGRATED) {
            errorReporter.reportError(targetConfig.compiler+" returns error code "+returnCode);
        }
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (stderr.toString().length() > 0 && fileConfig.getCompilerMode() == Mode.INTEGRATED) {
            generator.reportCommandErrors(stderr.toString());
        }
        return (returnCode == 0);
    }
    
    /**
     * Return a command to compile the specified C file using CMake.
     * This produces a C specific compile command.
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
                "The C target requires GCC >= 7 to compile the generated code. " +
                "Auto-compiling can be disabled using the \"no-compile: true\" target property.",
                true
        );
        
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
     * Return a command to compile the specified C file using a native compiler 
     * (generally gcc unless overriden by the user).
     * This produces a C specific compile command.
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param doNotLinkIfNoMain If true, the compile command will have a
     *  `-c` flag when there is no main reactor. If false, the compile command
     *  will never have a `-c` flag.
     * @param errorReporter Used to report errors to the user.
     */
    public ProcessBuilder compileCCommand(
            String fileToCompile, 
            boolean doNotLinkIfNoMain, 
            ErrorReporter errorReporter
    ) {
        ExecutionEnvironment env = generator.findCommandEnv(
            targetConfig.compiler, 
            "The C target requires GCC >= 7 to compile the generated code. " +
            "Auto-compiling can be disabled using the \"no-compile: true\" target property.",
            true
        );
        
        String cFilename = generator.getTargetFileName(fileToCompile);

        Path relativeSrcPath = fileConfig.getOutPath().relativize(
            fileConfig.getSrcGenPath().resolve(Paths.get(cFilename)));
        Path relativeBinPath = fileConfig.getOutPath().relativize(
            fileConfig.binPath.resolve(Paths.get(fileToCompile)));

        // NOTE: we assume that any C compiler takes Unix paths as arguments.
        String relSrcPathString = FileConfig.toUnixString(relativeSrcPath);
        String relBinPathString = FileConfig.toUnixString(relativeBinPath);
        
        // If there is no main reactor, then generate a .o file not an executable.
        if (generator.getMainDef() == null) {
            relBinPathString += ".o";
        }
        
        ArrayList<String> compileArgs = new ArrayList<String>();
        compileArgs.add(relSrcPathString);
        for (String file: targetConfig.compileAdditionalSources) {
            var relativePath = fileConfig.getOutPath().relativize(
                fileConfig.getSrcGenPath().resolve(Paths.get(file)));
            compileArgs.add(FileConfig.toUnixString(relativePath));
        }
        compileArgs.addAll(targetConfig.compileLibraries);

        // If threaded computation is requested, add a -pthread option.

        if (targetConfig.threads != 0 || targetConfig.tracing != null) {
            compileArgs.add("-pthread");
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            compileArgs.add("-DNUMBER_OF_WORKERS="+targetConfig.threads);
        }
        
        // Finally add the compiler flags in target parameters (if any)
        if (!targetConfig.compilerFlags.isEmpty()) {
            compileArgs.addAll(targetConfig.compilerFlags);
        }

        // Only set the output file name if it hasn't already been set
        // using a target property or Args line flag.
        if (!compileArgs.contains("-o")) {
            compileArgs.add("-o");
            compileArgs.add(relBinPathString);
        }
        
        // If there is no main reactor, then use the -c flag to prevent linking from occurring.
        // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
        // Then again, I think this only makes sense when we can do linking.
        // In any case, a warning is helpful to draw attention to the fact that no binary was produced.
        if (doNotLinkIfNoMain && generator.main == null) {
            compileArgs.add("-c"); // FIXME: revisit
            if (fileConfig.getCompilerMode() == Mode.STANDALONE) {
                errorReporter.reportError("ERROR: Did not output executable; no main reactor found.");
            }
        }
        return generator.createCommand(targetConfig.compiler, compileArgs, fileConfig.getOutPath(), env);
    }

}
