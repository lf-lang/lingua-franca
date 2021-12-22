package org.lflang.generator;

import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.CancelIndicator;

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
    /** Whether the requested build is required to be complete. */
    // FIXME: This private member is perhaps not right, but
    //  the way it gets used (in FileConfig.getCompilerMode)
    //  is already not right.
    private final boolean complete;
    /** The result of the code generation process. */
    private GeneratorResult result = null;
    /* ------------------------- PUBLIC METHODS -------------------------- */

    /** Initializes a context that cannot be cancelled. */
    public SlowIntegratedContext(boolean complete) {
        this(complete, CancelIndicator.NullImpl);
    }

    /**
     * Initializes the context of a generation process whose
     * cancellation is indicated by {@code cancelIndicator}
     * @param complete whether the requested build is
     *                 required to be complete
     * @param cancelIndicator the cancel indicator of the
     *                        code generation process to
     *                        which this corresponds
     */
    public SlowIntegratedContext(boolean complete, CancelIndicator cancelIndicator) {
        this.cancelIndicator = cancelIndicator;
        this.complete = complete;
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
}
