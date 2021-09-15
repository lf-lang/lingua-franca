package org.lflang.generator;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;

import org.lflang.ErrorReporter;

public class LanguageServerErrorReporter implements ErrorReporter {
    // FIXME: This is a placeholder implementation. Replace this stub
    //  with an error reporter that will send messages to the language
    //  client in the form of user-friendly diagnostics.

    boolean errorsOccurred = false;

    @Override
    public String reportError(String message) {
        System.err.println(message);
        errorsOccurred = true;
        return message;
    }

    @Override
    public String reportWarning(String message) {
        System.err.println(message);
        return message;
    }

    @Override
    public String reportError(EObject object, String message) {
        System.err.println("Error associated with EObject " + object + ": " + message);
        errorsOccurred = true;
        return message;
    }

    @Override
    public String reportWarning(EObject object, String message) {
        System.err.println("Warning associated with EObject " + object + ": " + message);
        return message;
    }

    @Override
    public String reportError(Path file, Integer line, String message) {
        System.err.println("Error associated with line " + line + ": " + message);
        errorsOccurred = true;
        return message;
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        System.err.println("Warning associated with line " + line + ": " + message);
        return message;
    }

    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }

    @Override
    public void reset() {
        errorsOccurred = false;
    }
}
