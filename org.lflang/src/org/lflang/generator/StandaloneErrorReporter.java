/*************
 * Copyright (c) 2019, The University of California at Berkeley. Copyright (c)
 * 2019, TU Dresden
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.validation.EObjectDiagnosticImpl;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;

import com.google.inject.Inject;

/**
 * An error reporter that forwards all messages to an {@link IssueCollector}.
 * They'll be sorted out later.
 */
public class StandaloneErrorReporter implements ErrorReporter {

    @Inject
    private IssueCollector collector;

    /**
     * Report a warning or error on the specified object
     * <p>
     * The caller should not throw an exception so execution can continue. This
     * will print the error message to stderr. If running in INTEGRATED mode
     * (within the Eclipse IDE), then this also adds a marker to the editor.
     *
     * @param message  The error message.
     * @param severity One of IMarker.SEVERITY_ERROR or IMarker.SEVERITY_WARNING
     * @param obj      The Ecore object, or null if it is not known.
     */
    private String reportWithNode(String message, Severity severity, EObject obj) {
        EObjectDiagnosticImpl diagnostic =
            new EObjectDiagnosticImpl(severity, null, message, obj, null, 0, null);

        Path path;
        try {
            path = FileConfig.toPath(obj.eResource());
        } catch (IOException ignored) {
            path = getFileNameBestEffort(obj);
        }

        LfIssue issue = new LfIssue(
            message,
            severity,
            diagnostic.getLine(),
            diagnostic.getColumn(),
            diagnostic.getLength(),
            path
        );

        collector.accept(issue);
        return "";
    }


    /** Best effort to get a fileName. May return null. */
    private static Path getFileNameBestEffort(EObject obj) {
        Path path;
        URI uri = obj.eResource().getURI();
        try {
            path = FileConfig.toPath(uri);
        } catch (IOException ioe) {
            String fString = uri.toFileString();
            path = fString != null ? Paths.get(fString) : null;
        }
        return path;
    }


    /**
     * Report a warning or error on the specified line of the specified file.
     * <p>
     * The caller should not throw an exception so execution can continue. This
     * will print the error message to stderr. If running in INTEGRATED mode
     * (within the Eclipse IDE), then this also adds a marker to the editor.
     *
     * @param message  The error message.
     * @param severity A severity
     * @param line     The line number or null if it is not known.
     * @param path     The file, or null if it is not known.
     */
    private String reportSimpleFileCtx(String message, Severity severity, Integer line, Path path) {
        LfIssue issue = new LfIssue(
            message,
            severity,
            line,
            null,
            null,
            path
        );

        collector.accept(issue);

        // Return a string that can be inserted into the generated code.
        return message;
    }


    @Override
    public String reportError(String message) {
        return reportSimpleFileCtx(message, Severity.ERROR, null, null);
    }


    @Override
    public String reportWarning(String message) {
        return reportSimpleFileCtx(message, Severity.WARNING, null, null);
    }


    @Override
    public String reportError(EObject obj, String message) {
        return reportWithNode(message, Severity.ERROR, obj);
    }


    @Override
    public String reportWarning(EObject obj, String message) {
        return reportWithNode(message, Severity.WARNING, obj);
    }


    @Override
    public String reportError(Path file, Integer line, String message) {
        return reportSimpleFileCtx(message, Severity.ERROR, line, file);
    }


    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return reportSimpleFileCtx(message, Severity.WARNING, line, file);
    }


    @Override
    public Boolean getErrorsOccurred() {
        return collector.getErrorsOccurred();
    }


    @Override
    public void reset() {
        collector.reset();
    }
}
