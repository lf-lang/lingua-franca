package org.lflang.generator;

import org.eclipse.core.resources.IResource;
import org.eclipse.emf.ecore.EObject;

public interface IErrorReporter {

    /**
     * Report a warning or error on the specified line of the specified resource.
     * <p>
     * The caller should not throw an exception so execution can continue.
     * This will print the error message to stderr.
     * If running in INTEGRATED mode (within the Eclipse IDE), then this also
     * adds a marker to the editor.
     *
     * @param message  The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param line     The line number or null if it is not known.
     * @param resource The resource, or null if it is not known.
     */
    String report(String message, Integer severity, Integer line, IResource resource);


    /**
     * Report an error.
     *
     * @param message The error message.
     */
    String reportError(String message);


    /**
     * Report a warning.
     *
     * @param message The warning message.
     */
    String reportWarning(String message);


    /**
     * Report an error on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    String reportError(EObject obj, String message);


    /**
     * Report a warning on the receiving parse tree object.
     *
     * @param obj     The parse tree object.
     * @param message The error message.
     */
    String reportWaring(EObject obj, String message);
}
