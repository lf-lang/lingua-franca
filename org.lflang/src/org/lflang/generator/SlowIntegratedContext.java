package org.lflang.generator;

import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.CancelIndicator;

public class SlowIntegratedContext implements IGeneratorContext {

    /**
     * The indicator that shows whether this generation
     * process is canceled.
     */
    private final CancelIndicator cancelIndicator;
    /** Whether the requested build is required to be complete. */
    private final boolean complete;
    // FIXME: This private member is perhaps not right, but
    //  the way it gets used (in FileConfig.getCompilerMode)
    //  is already not right.
    /* ------------------------- PUBLIC METHODS -------------------------- */

    /** Initializes a context that cannot be cancelled. */
    public SlowIntegratedContext(boolean complete) {
        this(CancelIndicator.NullImpl, complete);
    }

    /**
     * Initializes the context of a generation process whose
     * cancellation is indicated by {@code cancelIndicator}
     * @param cancelIndicator the cancel indicator of the
     *                        code generation process to
     *                        which this corresponds
     * @param complete whether the requested build is
     *                 required to be complete
     */
    public SlowIntegratedContext(CancelIndicator cancelIndicator, boolean complete) {
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
}
