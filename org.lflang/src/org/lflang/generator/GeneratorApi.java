package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;
import org.lflang.federated.FederatedGenerationHelper;

/**
 * Common supertype of {@link GeneratorBase} and {@link KtGeneratorBase}.
 */
public interface GeneratorApi extends FederatedGenerationHelper {
    // fixme this is transitional until we have a single GeneratorBase


    String getTargetTimeType();


    /**
     * Report an error.
     *
     * @param message The error message.
     */
    String reportError(final String message);


    /**
     * Report an error on the specified parse tree object.
     *
     * @param object  The parse tree object.
     * @param message The error message.
     */
    String reportError(final EObject object, final String message);


    /**
     * Report a warning on the specified parse tree object.
     *
     * @param object  The parse tree object.
     * @param message The error message.
     */
    String reportWarning(final EObject object, final String message);

}
