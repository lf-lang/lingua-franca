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
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.GeneratorCommandFactory;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.util.FileUtil;
import org.lflang.util.LFCommand;

/**
 * Responsible for creating and executing the necessary native command to compile code that is generated
 * by the CGenerator. This class invokes a compiler directly.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de}
 * @author{Matt Weber <matt.weber@berkeley.edu>}
 */
public class CCompiler {

    FileConfig fileConfig;
    TargetConfig targetConfig;
    ErrorReporter errorReporter;
    
    /** 
     * Indicate whether or not the compiler is in C++ mode 
     * In C++ mode, the compiler produces .cpp files instead
     * of .c files and uses a C++ compiler to compiler the code.
     */
    boolean CppMode = false;
    
    /**
     * A factory for compiler commands.
     */
    GeneratorCommandFactory commandFactory;
    
    /**
     * Create an instance of CCompiler.
     * 
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter Used to report errors.
     * @param CppMode Indicate if the compilation should happen in C++ mode
     */
    public CCompiler(
            TargetConfig targetConfig, 
            FileConfig fileConfig, 
            ErrorReporter errorReporter, 
            boolean CppMode
            ) {
        this(targetConfig, fileConfig, errorReporter);
        this.CppMode = CppMode;
    }
    
    /**
     * Create an instance of CCompiler.
     * 
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter Used to report errors.
     */
    public CCompiler(
            TargetConfig targetConfig, 
            FileConfig fileConfig, 
            ErrorReporter errorReporter) {
        this.fileConfig = fileConfig;
        this.targetConfig = targetConfig;
        this.errorReporter = errorReporter;
        this.commandFactory = new GeneratorCommandFactory(errorReporter, fileConfig);
        this.CppMode = false;
    }

    /** 
     * Run the C compiler by directly invoking a C compiler (gcc by default).
     * 
     * @param file The source file to compile without the .c extension.
     * @param noBinary If true, the compiler will create a .o output instead of a binary. 
     *  If false, the compile command will produce a binary.
     * @param generator An instance of GenratorBase, only used to report error line numbers
     *  in the Eclipse IDE.
     * 
     * @return true if compilation succeeds, false otherwise. 
     */
    public boolean runCCompiler(
        String file,
        boolean noBinary,
        GeneratorBase generator,
        LFGeneratorContext context
    ) throws IOException {
        if (noBinary && context.getMode() == LFGeneratorContext.Mode.STANDALONE) {
            errorReporter.reportError("Did not output executable; no main reactor found.");
        }
        LFCommand compile = compileCCommand(file, noBinary);
        if (compile == null) {
            return false;
        }
        
        int returnCode = compile.run(context.getCancelIndicator());

        if (returnCode != 0 && context.getMode() == LFGeneratorContext.Mode.STANDALONE) {
            errorReporter.reportError(targetConfig.compiler+" returns error code "+returnCode);
        }
        // For warnings (vs. errors), the return code is 0.
        // But we still want to mark the IDE.
        if (compile.getErrors().toString().length() > 0 && context.getMode() != LFGeneratorContext.Mode.STANDALONE) {
            generator.reportCommandErrors(compile.getErrors().toString());
        }
        
        if (returnCode == 0 && compile.getErrors().toString().length() == 0) {
            System.out.println("SUCCESS: Compiling generated code for "+ fileConfig.name +" finished with no errors.");
        }
        
        return (returnCode == 0);
    }
    
    /**
     * Return a command to compile the specified C file using a native compiler 
     * (generally gcc unless overridden by the user).
     * This produces a C specific compile command.
     * 
     * @param fileToCompile The C filename without the .c extension.
     * @param noBinary If true, the compiler will create a .o output instead of a binary. 
     *  If false, the compile command will produce a binary.
     */
    public LFCommand compileCCommand(
            String fileToCompile, 
            boolean noBinary
    ) {
        
        String cFilename = getTargetFileName(fileToCompile, CppMode);

        Path relativeSrcPath = fileConfig.getOutPath().relativize(
            fileConfig.getSrcGenPath().resolve(Paths.get(cFilename)));
        Path relativeBinPath = fileConfig.getOutPath().relativize(
            fileConfig.binPath.resolve(Paths.get(fileToCompile)));

        // NOTE: we assume that any C compiler takes Unix paths as arguments.
        String relSrcPathString = FileUtil.toUnixString(relativeSrcPath);
        String relBinPathString = FileUtil.toUnixString(relativeBinPath);
        
        // If there is no main reactor, then generate a .o file not an executable.
        if (noBinary) {
            relBinPathString += ".o";
        }
        
        ArrayList<String> compileArgs = new ArrayList<String>();
        compileArgs.add(relSrcPathString);
        for (String file: targetConfig.compileAdditionalSources) {
            var relativePath = fileConfig.getOutPath().relativize(
                fileConfig.getSrcGenPath().resolve(Paths.get(file)));
            compileArgs.add(FileUtil.toUnixString(relativePath));
        }
        compileArgs.addAll(targetConfig.compileLibraries);

        // Add compile definitions
        targetConfig.compileDefinitions.forEach( (key,value) -> {
            compileArgs.add("-D"+key+"="+value);
        });
        
        // If threaded computation is requested, add a -pthread option.
        if (targetConfig.threading || targetConfig.tracing != null) {
            compileArgs.add("-pthread");
            // If the LF program itself is threaded or if tracing is enabled, we need to define
            // NUMBER_OF_WORKERS so that platform-specific C files will contain the appropriate functions
            compileArgs.add("-DNUMBER_OF_WORKERS="+targetConfig.workers);
        }
        
        // Finally, add the compiler flags in target parameters (if any)
        compileArgs.addAll(targetConfig.compilerFlags);

        // Only set the output file name if it hasn't already been set
        // using a target property or Args line flag.
        if (!compileArgs.contains("-o")) {
            compileArgs.add("-o");
            compileArgs.add(relBinPathString);
        }
        
        // If there is no main reactor, then use the -c flag to prevent linking from occurring.
        // FIXME: we could add a `-c` flag to `lfc` to make this explicit in stand-alone mode.
        // Then again, I think this only makes sense when we can do linking.
        if (noBinary) {
            compileArgs.add("-c"); // FIXME: revisit
        }
        
        LFCommand command = commandFactory.createCommand(targetConfig.compiler, compileArgs, fileConfig.getOutPath());
        if (command == null) {
            errorReporter.reportError(
                "The C/CCpp target requires GCC >= 7 to compile the generated code. " +
                "Auto-compiling can be disabled using the \"no-compile: true\" target property.");
        }
        return command;
    
    }
    
    
    /** Produces the filename including the target-specific extension 
     * 
     * @param fileName The base name of the file without any extensions
     * @param CppMode  Indicate whether or not the compiler is in C++ mode 
     * In C++ mode, the compiler produces .cpp files instead
     * of .c files and uses a C++ compiler to compiler the code.
     */
    static String getTargetFileName(String fileName, boolean CppMode) {
        if (CppMode) {
            // If the C++ mode is enabled, use a .cpp extension
            return fileName + ".cpp";
        }
        return fileName + ".c";
    }
}
