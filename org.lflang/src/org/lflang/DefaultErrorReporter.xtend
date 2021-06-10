package org.lflang

import org.eclipse.emf.ecore.EObject
import java.nio.file.Path

/**
 * Simple implementation of the ErrorReport interface that simply prints to
 * standard out.
 */
class DefaultErrorReporter implements ErrorReporter {

    /**
     * Default error reporter that prints to standard out.
     */
    public static val DEFAULT = new DefaultErrorReporter()

    /**
     * Print the given error message.
     */
    override reportError(String message) {
        println("ERROR: " + message)
    }

    /**
     * Print the given error message.
     */
    override reportError(EObject object, String message) {
        println("ERROR: " + message)
    }

    /**
     * Print the given error message.
     */
    override reportError(Path file, Integer line, String message) {
        println("ERROR: " + message)
    }

    /**
     * Print the given warning message.
     */
    override reportWarning(String message) {
        println("WARNING: " + message)
    }

    /**
     * Print the given warning message.
     */
    override reportWarning(EObject object, String message) {
        println("WARNING: " + message)
    }

    /**
     * Print the given warning message.
     */
    override reportWarning(Path file, Integer line, String message) {
        println("WARNING: " + message)
    }

}
