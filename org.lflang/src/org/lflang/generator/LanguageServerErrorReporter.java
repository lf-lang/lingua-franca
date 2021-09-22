package org.lflang.generator;

import java.nio.file.Path;
import java.util.Optional;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;

import org.lflang.ErrorReporter;

public class LanguageServerErrorReporter implements ErrorReporter {

    /**
     * Represents a diagnostic severity level.
     */
    public enum Severity {
        ERROR, WARNING, INFO
    }

    /**
     * Functional interface describing a diagnostic
     * reporting channel such as
     * ValidationMessageAcceptor::acceptError.
     */
    private interface Channel {
        void apply(String message, EObject object, int offset, int length, String code, String...issueData);
    }

    /** The document for which this is a diagnostic acceptor. */
    private final EObject parseRoot;
    /** The object for which this is a wrapper. */
    private final ValidationMessageAcceptor acceptor;
    /** Whether errors occurred since the last reset. */
    boolean errorsOccurred = false;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Initializes a <code>DiagnosticAcceptor</code> for the
     * document whose parse tree root node is
     * <code>parseRoot</code>.
     * @param parseRoot the root of the AST of the document
     *                  for which this is an error reporter
     * @param acceptor the object for which this is a
     *                 wrapper
     */
    public LanguageServerErrorReporter(EObject parseRoot, ValidationMessageAcceptor acceptor) {
        this.parseRoot = parseRoot;
        this.acceptor = acceptor;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    @Override
    public String reportError(String message) {
        return acceptDiagnostic(Severity.ERROR, message);
    }

    @Override
    public String reportWarning(String message) {
        return acceptDiagnostic(Severity.WARNING, message);
    }

    @Override
    public String reportError(EObject object, String message) {
        return reportError(message);
    }

    @Override
    public String reportWarning(EObject object, String message) {
        return reportWarning(message);
    }

    @Override
    public String reportError(Path file, Integer line, String message) { // FIXME
        return acceptDiagnostic(Severity.ERROR, message, line != null ? line : 0);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) { // FIXME
        return acceptDiagnostic(Severity.WARNING, message, line != null ? line : 0);
    }

    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }

    @Override
    public void reset() {
        errorsOccurred = false;
    }

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     * @return a string that describes the diagnostic
     */
    public String acceptDiagnostic(Severity severity, String message, Position startPos, Position endPos) {
        if (severity == Severity.ERROR) errorsOccurred = true;
        int offset = startPos.getOffset(getText());
        // FIXME: This is a constant factor (typically ~2x) slower than it really needs to be, since the endPos
        //  is likely to be close to the startPos.
        int length = endPos.getOffset(getText()) - offset;
        getChannel(severity).apply(message, parseRoot, offset, length, null);
        return "" + severity + ": " + message;
    }

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @return a string that describes the diagnostic
     */
    public String acceptDiagnostic(Severity severity, String message) {
        return acceptDiagnostic(severity, message, 0);
    }

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param line the zero-based line number associated
     *             with the message
     * @return a string that describes the diagnostic
     */
    public String acceptDiagnostic(Severity severity, String message, int line) {
        Optional<String> firstLine = getLine(line);
        return acceptDiagnostic(
            severity,
            message,
            Position.fromZeroBased(0, 0),
            Position.fromZeroBased(0, firstLine.isEmpty() ? 1 : firstLine.get().length())
        );
    }

    /**
     * Reports an error regarding an LF document.
     * @param message the error message to be sent to the
     *                IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     */
    public String error(String message, Position startPos, Position endPos) {
        return acceptDiagnostic(Severity.ERROR, message, startPos, endPos);
    }

    /**
     * Reports a warning regarding an LF document.
     * @param message the warning message to be sent to the
     *                IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     */
    public String warning(String message, Position startPos, Position endPos) {
        return acceptDiagnostic(Severity.WARNING, message, startPos, endPos);
    }

    /**
     * Reports an info message regarding an LF document.
     * @param message the info message to be sent to the
     *                IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     */
    public String info(String message, Position startPos, Position endPos) {
        return acceptDiagnostic(Severity.INFO, message, startPos, endPos);
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /**
     * Returns the appropriate diagnostic reporting channel
     * for the given severity.
     * @param severity the severity of a diagnostic
     * @return the corresponding diagnostic reporting
     * channel
     */
    private Channel getChannel(Severity severity) {
        switch (severity) {
        case ERROR:
            return acceptor::acceptError;
        case WARNING:
            return acceptor::acceptWarning;
        default:
            return acceptor::acceptInfo;
        }
    }

    /**
     * Returns the text of the document for which this is an
     * error reporter.
     * @return the text of the document for which this is an
     * error reporter
     */
    private String getText() {
        return NodeModelUtils.getNode(parseRoot).getText();
    }

    /**
     * Returns the line at index <code>line</code> in the
     * document for which this is an error reporter.
     * @param line the zero-based line index
     * @return the line located at the given index
     */
    private Optional<String> getLine(int line) {
        return getText().lines().skip(line).findFirst();
    }
}
