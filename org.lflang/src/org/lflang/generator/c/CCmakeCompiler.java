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
