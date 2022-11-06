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

package org.lflang.cli;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.diagnostics.Severity;

import org.lflang.ErrorReporter;

import com.google.inject.Inject;

/**
 * An error reporter that forwards all messages to an {@link IssueCollector}.
 * They'll be sorted out later.
 */
public class StandaloneErrorReporter implements ErrorReporter {

    @Inject
    private StandaloneIssueAcceptor issueAcceptor;

    private String reportWithNode(String message, Severity severity, EObject obj) {
        issueAcceptor.accept(severity, message, obj, null, 0, null);
        return message;
    }

    private String reportSimpleFileCtx(String message, Severity severity, Integer line, Path path) {
        LfIssue issue = new LfIssue(message, severity, line, 1, line, 1, 0, path);
        issueAcceptor.accept(issue);
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
    public String reportInfo(String message) {
        return reportSimpleFileCtx(message, Severity.INFO, null, null);
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
    public String reportInfo(EObject obj, String message) {
        return reportWithNode(message, Severity.INFO, obj);
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
    public String reportInfo(Path file, Integer line, String message) {
        return reportSimpleFileCtx(message, Severity.INFO, line, file);
    }


    @Override
    public boolean getErrorsOccurred() {
        return issueAcceptor.getErrorsOccurred();
    }
}
