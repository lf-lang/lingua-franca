package org.lflang;

import org.eclipse.emf.ecore.EObject;

import java.nio.file.Path;

/**
 * Simple implementation of the ErrorReport interface that simply prints to
 * standard out.
 */
public class DefaultErrorReporter implements ErrorReporter {

    private boolean errorsOccurred = false;

    private String println(String s) {
        System.out.println(s);
        return s;
    }

    @Override
    public String reportError(String message) {
        errorsOccurred = true;
        return println("ERROR: " + message);
    }

    @Override
    public String reportError(EObject object, String message) {
        errorsOccurred = true;
        return println("ERROR: " + message);
    }

    @Override
    public String reportError(Path file, Integer line, String message) {
        errorsOccurred = true;
        return println("ERROR: " + message);
    }

    @Override
    public String reportWarning(String message) {
        return println("WARNING: " + message);
    }

    @Override
    public String reportInfo(String message) {
        return println("INFO: " + message);
    }

    @Override
    public String reportWarning(EObject object, String message) {
        return println("WARNING: " + message);
    }

    @Override
    public String reportInfo(EObject object, String message) {
        return println("INFO: " + message);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return println("WARNING: " + message);
    }

    @Override
    public String reportInfo(Path file, Integer line, String message) {
        return println("INFO: " + message);
    }

    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }
}
