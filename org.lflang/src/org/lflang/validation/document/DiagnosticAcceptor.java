package org.lflang.validation.document;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;

/**
 * Accepts diagnostics (also called validation messages) and
 * relays them on to the IDE.
 */
class DiagnosticAcceptor {

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

    /*
    Implementation note. This class is little more than a
    wrapper class for
    org.eclipse.xtext.validation.ValidationMessageAcceptor.
    This code could be considered bloat. It is justified by
    the facts that
    1. ValidationMessageAcceptor requires more data than is
    actually being produced here. If it were used in this
    package, then meaningless null arguments would clutter
    the code, and an EObject would have to be passed around
    unnecessarily so that it could be passed to
    ValidationMessageAcceptor.
    1. Tight coupling with Xtext classes is not desired. It
    hinders refactoring.

    Why not pass a function (say, LFValidator::error)
    instead? If that approach were taken, then several other
    methods (info, warning, and overloadings thereof) would
    also be desired.
     */
    /** The document for which this is a diagnostic acceptor. */
    private final LFDocument document;
    /** The object for which this is a wrapper. */
    private final ValidationMessageAcceptor acceptor;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Initializes a <code>DiagnosticAcceptor</code> for the
     * document whose parse tree root node is
     * <code>parseRoot</code>.
     * @param document the document for which this is a
     *                 diagnostic acceptor
     * @param acceptor the object for which this is a wrapper
     */
    public DiagnosticAcceptor(LFDocument document, ValidationMessageAcceptor acceptor) {
        this.document = document;
        this.acceptor = acceptor;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     */
    public void acceptDiagnostic(Severity severity, String message, Position startPos, Position endPos) {
        int offset = document.getOffset(startPos);
        // FIXME: This is a constant factor (typically ~2x) slower than it really needs to be, since the endPos
        //  is likely to be close to the startPos; however, it keeps the interface of LFDocument simple, which
        //  might be worth the performance hit.
        int length = document.getOffset(endPos) - offset;
        getChannel(severity).apply(message, document.getParseRoot(), offset, length, null);
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
    public void error(String message, Position startPos, Position endPos) {
        acceptDiagnostic(Severity.ERROR, message, startPos, endPos);
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
    public void warning(String message, Position startPos, Position endPos) {
        acceptDiagnostic(Severity.WARNING, message, startPos, endPos);
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
    public void info(String message, Position startPos, Position endPos) {
        acceptDiagnostic(Severity.INFO, message, startPos, endPos);
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
}
