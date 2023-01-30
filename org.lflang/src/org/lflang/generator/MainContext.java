package org.lflang.generator;

import java.io.IOException;
import java.util.Objects;
import java.util.Properties;
import java.util.function.Function;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.util.RuntimeIOException;

import org.lflang.DefaultErrorReporter;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.IntegratedBuilder.ReportProgress;

/**
 * A {@code MainContext} is an {@code LFGeneratorContext} that is
 * not nested in any other generator context. There is one
 * {@code MainContext} for every build process.
 *
 * @author Peter Donovan
 */
public class MainContext implements LFGeneratorContext {
    /**
     * The indicator that shows whether this build
     * process is canceled.
     */
    private final CancelIndicator cancelIndicator;
    /** The {@code ReportProgress} function of {@code this}. */
    private final ReportProgress reportProgress;

    private final FileConfig fileConfig;

    /** Whether the requested build is required to be complete. */
    private final Mode mode;

    private TargetConfig targetConfig;

    /** The result of the code generation process. */
    private GeneratorResult result = null;
    private final Properties args;
    private final ErrorReporter errorReporter;

    /**
     * Initialize the context of a build process whose cancellation is
     * indicated by {@code cancelIndicator}
     * @param mode The mode of this build process.
     * @param cancelIndicator The cancel indicator of the code generation
     *                        process to which this corresponds.
     */
    public MainContext(Mode mode, Resource resource, IFileSystemAccess2 fsa, CancelIndicator cancelIndicator) {
        this(
            mode, cancelIndicator, (message, completion) -> {}, new Properties(), resource, fsa,
            fg -> new DefaultErrorReporter()
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
     * @param resource ...
     * @param fsa ...
     * @param constructErrorReporter A function that constructs the appropriate
     *                               error reporter for the given FileConfig.
     */
    public MainContext(
        Mode mode,
        CancelIndicator cancelIndicator,
        ReportProgress reportProgress,
        Properties args,
        Resource resource,
        IFileSystemAccess2 fsa,
        Function<FileConfig, ErrorReporter> constructErrorReporter
    ) {
        this.mode = mode;
        this.cancelIndicator = cancelIndicator == null ? () -> false : cancelIndicator;
        this.reportProgress = reportProgress;
        this.args = args;

        try {
            var useHierarchicalBin = args.containsKey("hierarchical-bin") && Boolean.parseBoolean(args.getProperty("hierarchical-bin"));
            fileConfig = Objects.requireNonNull(LFGenerator.createFileConfig(resource, FileConfig.getSrcGenRoot(fsa), useHierarchicalBin));
        } catch (IOException e) {
            throw new RuntimeIOException("Error during FileConfig instantiation", e);
        }

        this.errorReporter = constructErrorReporter.apply(this.fileConfig);

        loadTargetConfig();
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
    public ErrorReporter getErrorReporter() {
        return errorReporter;
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
    public FileConfig getFileConfig() {
        return this.fileConfig;
    }

    @Override
    public TargetConfig getTargetConfig() {
        return this.targetConfig;
    }

    @Override
    public void reportProgress(String message, int percentage) {
        reportProgress.apply(message, percentage);
    }

    /**
     * Load the target configuration based on the contents of the resource.
     * This is done automatically upon instantiation of the context, but
     * in case the resource changes (e.g., due to an AST transformation),
     * this method can be called to reload to ensure that the changes are
     * reflected in the target configuration.
     */
    public void loadTargetConfig() {
        this.targetConfig = GeneratorUtils.getTargetConfig(
            args, GeneratorUtils.findTarget(fileConfig.resource), errorReporter
        );
    }
}
