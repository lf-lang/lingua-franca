package org.lflang.generator;

import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IGeneratorContext;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig.Mode;
import org.lflang.util.LFCommand;

/**
 * An {@code LFGeneratorContext} is the context of a Lingua Franca build process.
 */
public interface LFGeneratorContext extends IGeneratorContext {

    /**
     * Return the mode of operation, which indicates how the compiler has been invoked 
     * (e.g., from within Epoch, from the command line, or via a Language Server).
     */
    Mode getMode();

    /**
     * Returns the arguments of this.
     */
    Properties getArgs();

    /**
     * Return whether the bin directory should be hierarchical.
     * @return whether the bin directory should be hierarchical
     */
    boolean isHierarchicalBin();

    /**
     * Construct the appropriate error reporter for {@code fileConfig}.
     * @param fileConfig The {@code FileConfig} used by a build process.
     * @return the appropriate error reporter for {@code fileConfig}
     */
    ErrorReporter constructErrorReporter(FileConfig fileConfig);

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

    /**
     * Report the progress of a build.
     * @param message A message for the LF programmer to read.
     * @param percentage The approximate percent completion of the build.
     */
    void reportProgress(String message, int percentage);

    /**
     * Informs the context of the result of its build, if applicable.
     * @param status The status of the result.
     * @param execName The name of the executable produced by this code
     * generation process, or {@code null} if no executable was produced.
     * @param binPath The directory containing the executable (if applicable)
     * @param codeMaps The generated files and their corresponding code maps.
     * @param interpreter The interpreter needed to run the executable, if
     *                    applicable.
     */
    default void finish(
        GeneratorResult.Status status,
        String execName,
        Path binPath,
        Map<Path, CodeMap> codeMaps,
        String interpreter
    ) {
        if (execName != null && binPath != null) {
            Path executable = binPath.resolve(execName);
            Path start = binPath.getRoot();
            Path end = null;
            for (Path segment: binPath) {
                if (end == null) {
                    if (start.resolve("src").toFile().exists()) end = segment;
                    else start = start.resolve(segment);
                } else {
                    end = end.resolve(segment);
                    if (start.resolve(end).resolve("src").toFile().exists()) {
                        start = start.resolve(end);
                        end = null;
                    }
                }
            }
            if (end == null) end = Path.of(".");
            String relativeExecutable = end.resolve(execName).toString();
            LFCommand command = interpreter != null ? LFCommand.get(interpreter, List.of(relativeExecutable), start) :
                LFCommand.get(relativeExecutable, List.of(), start);
            finish(new GeneratorResult(status, executable, command, codeMaps));
        } else {
            finish(new GeneratorResult(status, null, null, codeMaps));
        }
    }

    /**
     * Informs the context of the result of its build, if applicable.
     * @param status The status of the result.
     * @param execName The name of the executable produced by this code
     * generation process, or {@code null} if no executable was produced.
     * @param binPath The directory containing the executable (if applicable)
     * @param codeMaps The generated files and their corresponding code maps.
     */
    default void finish(
        GeneratorResult.Status status,
        String execName,
        Path binPath,
        Map<Path, CodeMap> codeMaps
    ) {
        finish(status, execName, binPath, codeMaps, null);
    }

    /**
     * Informs the context of that its build finished unsuccessfully.
     */
    default void unsuccessfulFinish() {
        finish(getCancelIndicator().isCanceled() ? GeneratorResult.CANCELLED : GeneratorResult.FAILED);
    }

    /**
     * Return the {@code LFGeneratorContext} that best describes the given {@code context} when
     * building {@code Resource}.
     * @param context The context of a Lingua Franca build process.
     * @param resource The resource being built.
     * @return The {@code LFGeneratorContext} that best describes the given {@code context} when
     * building {@code Resource}.
     */
    static LFGeneratorContext lfGeneratorContextOf(IGeneratorContext context, Resource resource) {
        if (context instanceof LFGeneratorContext) return (LFGeneratorContext) context;
        if (resource.getURI().isPlatform()) return new OuterContext(
            Mode.EPOCH, context.getCancelIndicator(), (m, p) -> {}, new Properties(), false,
            EclipseErrorReporter::new
        );
        return new OuterContext(Mode.LSP_FAST, context.getCancelIndicator());
    }
}
