package org.lflang;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.generator.Position;

/**
 * Interface for reporting errors.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Christian Menard
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
     * Report an informational message.
     *
     * @param message The message to report
     * @return a string that describes the error
     */
    String reportInfo(String message);


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
     * Report an informational message on the specified parse tree object.
     *
     * @param object The parse tree object.
     * @param message The informational message
     * @return a string that describes the info
     */
    String reportInfo(EObject object, String message);


    /**
     * Report an error at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The one-based line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the error.
     */
    String reportError(Path file, Integer line, String message);


    /**
     * Report a warning at the specified line within a file.
     *
     * @param message The error message.
     * @param line    The one-based line number to report at.
     * @param file    The file to report at.
     * @return a string that describes the warning.
     */
    String reportWarning(Path file, Integer line, String message);


    /**
     * Report an informational message at the specified line within a file.
     *
     * @param file The file to report at.
     * @param line The one-based line number to report at.
     * @param message The error message.
     * @return
     */
    String reportInfo(Path file, Integer line, String message);

    /**
     * Report a message of severity {@code severity}.
     * @param file The file to which the message pertains, or {@code null} if the file is unknown.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @return a string that describes the diagnostic
     */
    default String report(Path file, DiagnosticSeverity severity, String message) {
        switch (severity) {
        case Error:
            return reportError(message);
        case Warning:
        case Hint:
        case Information:
            return reportInfo(message);
        default:
            return reportWarning(message);
        }
    }

    /**
     * Report a message of severity {@code severity} that
     * pertains to line {@code line} of an LF source file.
     * @param file The file to which the message pertains, or {@code null} if the file is unknown.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param line the one-based line number associated
     *             with the message
     * @return a string that describes the diagnostic
     */
    default String report(Path file, DiagnosticSeverity severity, String message, int line) {
        switch (severity) {
        case Error:
            return reportError(file, line, message);
        case Warning:
        case Hint:
        case Information:
            return reportInfo(file, line, message);
        default:
            return reportWarning(file, line, message);
        }
    }

    /**
     * Report a message of severity {@code severity} that
     * pertains to the range [{@code startPos}, {@code endPos})
     * of an LF source file.
     * @param file The file to which the message pertains, or {@code null} if the file is unknown.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     * @return a string that describes the diagnostic
     */
    default String report(Path file, DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
        return report(file, severity, message, startPos.getOneBasedLine());
    }

    /**
     * Check if errors where reported.
     *
     * @return true if errors where reported
     */
    boolean getErrorsOccurred();
}
