package org.lflang;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;

/**
 * Interface for reporting errors.
 *
 * @author Edward A. Lee <eal@berkeley.edu>
 * @author Marten Lohstroh <marten@berkeley.edu>
 * @author Christian Menard <christian.menard@tu-dresden.de>
 */
public interface ErrorReporter {

    /**
     * Report an error.
     *
     * @param message The error message.
     * @return a string that describes the error.
     */
    String reportError(String message);


    /**
     * Report a warning.
     *
     * @param message The warning message.
     * @return a string that describes the warning.
     */
    String reportWarning(String message);


    /**
     * Report an error on the specified parse tree object.
     *
     * @param object  The parse tree object.
     * @param message The error message.
     * @return a string that describes the error.
     */
    String reportError(EObject object, String message);


    /**
     * Report a warning on the specified parse tree object.
     *
     * @param object  The parse tree object.
     * @param message The error message.
     * @return a string that describes the warning.
     */
    String reportWarning(EObject object, String message);


    /**
     * Report an error at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the error.
     */
    String reportError(Path file, Integer line, String message);


    /**
     * Report a warning at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the warning.
     */
    String reportWarning(Path file, Integer line, String message);


    /**
     * Check if errors where reported.
     *
     * @return true if errors where reported
     */
    boolean getErrorsOccurred();


    /**
     * Reset the error reporter.
     * After this, getErrorsOccurred() returns false. This
     * will also clean any IDE markers if the reporter sets
     * such markers.
     */
    void reset();
}
