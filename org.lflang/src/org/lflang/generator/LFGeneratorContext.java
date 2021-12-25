package org.lflang.generator;

import java.util.Properties;
import java.util.function.Function;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.DefaultErrorReporter;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig.Mode;
import org.lflang.generator.IntegratedBuilder.ReportProgress;

/**
 * An {@code LFGeneratorContext} is the context of a Lingua Franca build process.
 */
public class LFGeneratorContext implements IGeneratorContext {

    /**
     * The indicator that shows whether this build
     * process is canceled.
     */
    private final CancelIndicator cancelIndicator;
    /** The {@code ReportProgress} function of {@code this}. */
    private final ReportProgress reportProgress;
    /** Whether the requested build is required to be complete. */
    private final Mode mode;
    /** The result of the code generation process. */
    private GeneratorResult result = null;
    private final Properties args;
    private final boolean hierarchicalBin;  // FIXME: The interface would be simpler if this were part of {@code args}, and in addition, a potentially useful feature would be exposed to the user.
    private final Function<FileConfig, ErrorReporter> constructErrorReporter;
    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Initializes the context of a build process whose cancellation is
     * indicated by {@code cancelIndicator}
     * @param mode The mode of this build process.
     * @param cancelIndicator The cancel indicator of the code generation
     *                        process to which this corresponds.
     */
    public LFGeneratorContext(Mode mode, CancelIndicator cancelIndicator) {
        this(
            mode, cancelIndicator, (message, completion) -> {}, new Properties(), false,
            fileConfig -> new DefaultErrorReporter()
        );
    }

    /**
     * Initializes the context of a build process whose cancellation is
     * indicated by {@code cancelIndicator}
     * @param mode The mode of this build process.
     * @param cancelIndicator The cancel indicator of the code generation
     *                        process to which this corresponds.
     * @param reportProgress The {@code ReportProgress} function of
     *                       {@code this}.
     * @param args Any arguments that may be used to affect the product of the
     *             build.
     * @param hierarchicalBin Whether the bin directory should be structured
     *                        hierarchically.
     * @param constructErrorReporter A function that constructs the appropriate
     *                               error reporter for the given FileConfig.
     */
    public LFGeneratorContext(
        Mode mode,
        CancelIndicator cancelIndicator,
        ReportProgress reportProgress,
        Properties args,
        boolean hierarchicalBin,
        Function<FileConfig, ErrorReporter> constructErrorReporter
    ) {
        this.mode = mode;
        this.cancelIndicator = cancelIndicator;
        this.reportProgress = reportProgress;
        this.args = args;
        this.hierarchicalBin = hierarchicalBin;
        this.constructErrorReporter = constructErrorReporter;
    }

    @Override
    public CancelIndicator getCancelIndicator() {
        return cancelIndicator;
    }

    /**
     * Returns the mode of this.
     */
    public Mode getMode() {
        return mode;
    }

    /**
     * Returns the arguments of this.
     */
    public Properties getArgs() {
        return args;
    }

    /**
     * Returns whether the bin directory should be hierarchical.
     * @return whether the bin directory should be hierarchical
     */
    public boolean isHierarchicalBin() {
        return hierarchicalBin;
    }

    /**
     * Constructs the appropriate error reporter for {@code fileConfig}.
     * @param fileConfig The {@code FileConfig} used by a build process.
     * @return the appropriate error reporter for {@code fileConfig}
     */
    public ErrorReporter constructErrorReporter(FileConfig fileConfig) {
        return constructErrorReporter.apply(fileConfig);
    }

    /**
     * Marks the code generation process performed in this
     * context as finished with the result {@code result}.
     * @param result The result of the code generation
     *               process that was performed in this
     *               context.
     */
    public void finish(GeneratorResult result) {
        if (this.result != null) throw new IllegalStateException("A code generation process can only have one result.");
        this.result = result;
    }

    /**
     * Returns the result of the code generation process that was performed in
     * this context.
     * @return the result of the code generation process that was performed in
     * this context
     */
    public GeneratorResult getResult() {
        return result != null ? result : GeneratorResult.NOTHING;
    }

    /**
     * Reports the progress of a build.
     * @param message A message for the LF programmer to read.
     * @param percentage The approximate percent completion of the build.
     */
    public void reportProgress(String message, int percentage) {
        reportProgress.apply(message, percentage);
    }

    /**
     * Returns the {@code LFGeneratorContext} whose behavior
     * and state best approximate that of {@code context}.
     * @param context The context of a Lingua Franca build
     *                process.
     * @param resource The resource being built.
     * @return The {@code LFGeneratorContext} whose behavior
     * and state best approximate that of {@code context}.
     */
    public static LFGeneratorContext lfGeneratorContextOf(IGeneratorContext context, Resource resource) {
        if (context instanceof LFGeneratorContext) return (LFGeneratorContext) context;
        if (resource.getURI().isPlatform()) return new LFGeneratorContext(
            Mode.EPOCH, context.getCancelIndicator(), (m, p) -> {}, new Properties(), false,
            EclipseErrorReporter::new
        );
        return new LFGeneratorContext(Mode.LSP_FAST, context.getCancelIndicator());
    }
}
