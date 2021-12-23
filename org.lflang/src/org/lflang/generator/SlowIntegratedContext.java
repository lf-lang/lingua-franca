package org.lflang.generator;

import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.generator.IntegratedBuilder.ReportProgress;
import org.lflang.util.LFCommand;

/**
 * A {@code SlowIntegratedContext} is a context in which the
 * user of an IDE/text editor expects the code generator to
 * at least partially perform its function, even if that
 * means an instantaneous response is impossible.
 */
public class SlowIntegratedContext implements IGeneratorContext {

    /**
     * The indicator that shows whether this generation
     * process is canceled.
     */
    private final CancelIndicator cancelIndicator;
    /** The {@code ReportProgress} function of {@code this}. */
    private final ReportProgress reportProgress;
    /** Whether the requested build is required to be complete. */
    // FIXME: This private member is perhaps not right, but
    //  the way it gets used (in FileConfig.getCompilerMode)
    //  is already not right.
    private final boolean complete;
    /** The result of the code generation process. */
    private GeneratorResult result = null;
    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Initializes the context of a generation process whose
     * cancellation is indicated by {@code cancelIndicator}
     * @param complete Whether the requested build is
     *                 required to be complete.
     * @param cancelIndicator The cancel indicator of the
     *                        code generation process to
     *                        which this corresponds.
     * @param reportProgress The {@code ReportProgress}
     *                       function of {@code this}.
     */
    public SlowIntegratedContext(boolean complete, CancelIndicator cancelIndicator, ReportProgress reportProgress) {
        this.complete = complete;
        this.cancelIndicator = cancelIndicator;
        this.reportProgress = reportProgress;
    }

    @Override
    public CancelIndicator getCancelIndicator() {
        return cancelIndicator;
    }

    /**
     * Returns whether the requested build is required to be
     * complete.
     * @return whether the requested build is required to be
     * complete
     */
    public boolean getMustBeComplete() {
        return complete;
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
     * Informs the context of the result of its build, if applicable.
     * @param context The context in which the build was performed.
     * @param status The status of the result.
     * @param execName The name of the executable produced by this code
     * generation process, or {@code null} if no executable was produced.
     * @param binPath The directory containing the executable (if applicable)
     */
    public static void finish(
        IGeneratorContext context,
        GeneratorResult.Status status,
        String execName,
        Path binPath,
        Map<Path, CodeMap> codeMaps
    ) {
        finish(context, new GeneratorResult(
            status,
            execName == null ? null : binPath.resolve(execName),
            LFCommand.get("." + File.separator + execName, List.of(), binPath),
            codeMaps
        ));
    }

    /**
     * Informs the context of the result of its build, if applicable.
     * @param context The context in which the build was performed.
     * @param result The result of the build.
     */
    public static void finish(IGeneratorContext context, GeneratorResult result) {
        if (context instanceof SlowIntegratedContext) {
            reportProgress(context, "Build complete.", 100);
            ((SlowIntegratedContext) context).finish(result);
        }
    }

    /**
     * Informs the context of the current progress of its build,
     * if applicable.
     * @param context The context of a build.
     * @param message A message about the build's progress.
     * @param percentage The build's percent completion.
     */
    public static void reportProgress(IGeneratorContext context, String message, int percentage) {
        if (context instanceof SlowIntegratedContext) {
            ((SlowIntegratedContext) context).reportProgress.apply(message, percentage);
        }
    }
}
