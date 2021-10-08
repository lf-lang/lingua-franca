package org.lflang.generator;

import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.util.CancelIndicator;

public class SlowIntegratedContext implements IGeneratorContext {

    /**
     * The indicator that shows whether this generation
     * process is canceled.
     */
    private final CancelIndicator cancelIndicator;

    /* ------------------------- PUBLIC METHODS -------------------------- */

    /** Initializes a context that cannot be cancelled. */
    public SlowIntegratedContext() {
        this(CancelIndicator.NullImpl);
    }

    /**
     * Initializes the context of a generation process whose
     * cancellation is indicated by <code>cancelIndicator
     * </code>
     * @param cancelIndicator the cancel indicator of the
     *                        code generation process to
     *                        which this corresponds
     */
    public SlowIntegratedContext(CancelIndicator cancelIndicator) {
        this.cancelIndicator = cancelIndicator;
    }

    @Override
    public CancelIndicator getCancelIndicator() {
        return cancelIndicator;
    }
}
