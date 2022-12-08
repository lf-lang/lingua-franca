package org.lflang.generator;

import java.util.Properties;
import java.util.function.Function;

import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.DefaultErrorReporter;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.IntegratedBuilder.ReportProgress;

/**
 * A {@code MainContext} is an {@code LFGeneratorContext} that is
 * not nested in any other generator context. There is one
 * {@code MainContext} for every build process.
 *
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
public class MainContext implements LFGeneratorContext {
    
    /**
     * This constructor will be set by the LF plugin, if the generator is running in Epoch.
     */
    public static Function<FileConfig, ErrorReporter> EPOCH_ERROR_REPORTER_CONSTRUCTOR = null;
    
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
    private boolean hasConstructedErrorReporter;

    /**
     * Initialize the context of a build process whose cancellation is
     * indicated by {@code cancelIndicator}
     * @param mode The mode of this build process.
     * @param cancelIndicator The cancel indicator of the code generation
     *                        process to which this corresponds.
     */
    public MainContext(Mode mode, CancelIndicator cancelIndicator) {
        this(
            mode, cancelIndicator, (message, completion) -> {}, new Properties(), false,
            (mode == Mode.EPOCH && EPOCH_ERROR_REPORTER_CONSTRUCTOR != null) ?
                    EPOCH_ERROR_REPORTER_CONSTRUCTOR :
                    fileConfig -> new DefaultErrorReporter()
        );
    }

    /**
     * Initialize the context of a build process whose cancellation is
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
    public MainContext(
        Mode mode,
        CancelIndicator cancelIndicator,
        ReportProgress reportProgress,
        Properties args,
        boolean hierarchicalBin,
        Function<FileConfig, ErrorReporter> constructErrorReporter
    ) {
        this.mode = mode;
        this.cancelIndicator = cancelIndicator == null ? () -> false : cancelIndicator;
        this.reportProgress = reportProgress;
        this.args = args;
        this.hierarchicalBin = hierarchicalBin;
        this.constructErrorReporter = constructErrorReporter;
        this.hasConstructedErrorReporter = false;
    }

    @Override
    public CancelIndicator getCancelIndicator() {
        return cancelIndicator;
    }

    @Override
    public Mode getMode() {
        return mode;
    }

    @Override
    public Properties getArgs() {
        return args;
    }

    @Override
    public boolean useHierarchicalBin() {
        return hierarchicalBin;
    }

    @Override
    public ErrorReporter constructErrorReporter(FileConfig fileConfig) {
        if (hasConstructedErrorReporter) {
            throw new IllegalStateException("Only one error reporter should be constructed for a given context.");
        }
        hasConstructedErrorReporter = true;
        return constructErrorReporter.apply(fileConfig);
    }

    @Override
    public void finish(GeneratorResult result) {
        if (this.result != null) throw new IllegalStateException("A code generation process can only have one result.");
        this.result = result;
        reportProgress(result.getUserMessage(), 100);
    }

    @Override
    public GeneratorResult getResult() {
        return result != null ? result : GeneratorResult.NOTHING;
    }

    @Override
    public void reportProgress(String message, int percentage) {
        reportProgress.apply(message, percentage);
    }
}
