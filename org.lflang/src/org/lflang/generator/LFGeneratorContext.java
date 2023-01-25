package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;
import java.util.Objects;
import java.util.Properties;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.RuntimeIOException;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;

/**
 * An {@code LFGeneratorContext} is the context of a Lingua Franca build process.
 * It is the point of communication between a build process and the environment
 * in which it is executed.
 *
 * @author Peter Donovan
 */
public interface LFGeneratorContext extends IGeneratorContext {

    /**
     * Enumeration of keys used to parameterize the build process.
     */
    enum BuildParm {
        BUILD_TYPE("The build type to use"),
        CLEAN("Clean before building."),
        EXTERNAL_RUNTIME_PATH("Specify an external runtime library to be used by the compiled binary."),
        FEDERATED("Treat main reactor as federated."),
        HELP("Display this information."),
        LOGGING("The logging level to use by the generated binary"),
        LINT("Enable or disable linting of generated code."),
        NO_COMPILE("Do not invoke target compiler."),
        OUTPUT_PATH("Specify the root output directory."),
        QUIET("Suppress output of the target compiler and other commands"),
        RTI("Specify the location of the RTI."),
        RUNTIME_VERSION("Specify the version of the runtime library used for compiling LF programs."),
        SCHEDULER("Specify the runtime scheduler (if supported)."),
        TARGET_COMPILER("Target compiler to invoke."),
        THREADING("Specify whether the runtime should use multi-threading (true/false)."),
        VERSION("Print version information."),
        WORKERS("Specify the default number of worker threads.");

        public final String description;

        BuildParm(String description) {
            this.description = description;
        }

        /**
         * Return the string to use as the key to store a value relating to this parameter.
         */
        public String getKey() {
            return this.name().toLowerCase().replace('_', '-');
        }

        /**
         * Return the value corresponding to this parameter or `null` if there is none.
         * @param context The context passed to the code generator.
         */
        public String getValue(LFGeneratorContext context) {
            return context.getArgs().getProperty(this.getKey());
        }
    }


    enum Mode {
        STANDALONE,
        EPOCH,
        LSP_FAST,
        LSP_MEDIUM,
        LSP_SLOW,
        UNDEFINED
    }

    /**
     * Return the mode of operation, which indicates how the compiler has been invoked 
     * (e.g., from within Epoch, from the command line, or via a Language Server).
     */
    Mode getMode();

    /**
     * Return any arguments that will override target properties.
     */
    Properties getArgs();

    /**
     * Get the error reporter for this context; construct one if it hasn't been
     * constructed yet.
     */
    ErrorReporter getErrorReporter();

    /**
     * Mark the code generation process performed in this
     * context as finished with the result {@code result}.
     * @param result The result of the code generation
     *               process that was performed in this
     *               context.
     */
    void finish(GeneratorResult result);

    /**
     * Return the result of the code generation process that was performed in
     * this context.
     * @return the result of the code generation process that was performed in
     * this context
     */
    GeneratorResult getResult();

    FileConfig getFileConfig();

    TargetConfig getTargetConfig();

    /**
     * Report the progress of a build.
     * @param message A message for the LF programmer to read.
     * @param percentage The approximate percent completion of the build.
     */
    void reportProgress(String message, int percentage);

    /**
     * Conclude this build and record the result if necessary.
     * @param status The status of the result.
     * @param codeMaps The generated files and their corresponding code maps.
     */
    default void finish(
        GeneratorResult.Status status,
        Map<Path, CodeMap> codeMaps
    ) {
        finish(new GeneratorResult(status, this, codeMaps));
    }

    /**
     * Conclude this build and record that it was unsuccessful.
     */
    default void unsuccessfulFinish() {
        finish(
            getCancelIndicator() != null && getCancelIndicator().isCanceled() ?
            GeneratorResult.CANCELLED : GeneratorResult.FAILED
        );
    }

    /**
     * Return the {@code LFGeneratorContext} that best describes the given {@code context} when
     * building {@code Resource}.
     * @param resource
     * @param fsa
     * @param context The context of a Lingua Franca build process.
     * @return The {@code LFGeneratorContext} that best describes the given {@code context} when
     * building {@code Resource}.
     */
    static LFGeneratorContext lfGeneratorContextOf(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
        if (context instanceof LFGeneratorContext) return (LFGeneratorContext) context;

        if (resource.getURI().isPlatform()) return new MainContext(
            Mode.EPOCH, context.getCancelIndicator(), (m, p) -> {}, new Properties(),
            resource, fsa, EclipseErrorReporter::new
        );

        return new MainContext(Mode.LSP_FAST, resource, fsa, context.getCancelIndicator());
    }
}
