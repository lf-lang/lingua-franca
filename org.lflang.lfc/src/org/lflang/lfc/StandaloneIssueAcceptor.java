package org.lflang.lfc;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.validation.EObjectDiagnosticImpl;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;

import org.lflang.FileConfig;

import com.google.inject.Inject;

/**
 *
 */
public class StandaloneIssueAcceptor implements ValidationMessageAcceptor {

    @Inject
    private IssueCollector collector;


    boolean getErrorsOccurred() {
        return collector.getErrorsOccurred();
    }


    void reset() {
        collector.reset();
    }


    void accept(LfIssue lfIssue) {
        collector.accept(lfIssue);
    }


    void accept(Severity severity, String message, EObject object, EStructuralFeature feature, int index, String code, String... issueData) {
        EObjectDiagnosticImpl diagnostic =
            new EObjectDiagnosticImpl(severity, code, message, object, feature, index, issueData);

        LfIssue lfIssue = new LfIssue(
            message,
            severity,
            diagnostic.getLine(),
            diagnostic.getColumn(),
            diagnostic.getLineEnd(),
            diagnostic.getColumnEnd(),
            diagnostic.getLength(),
            getPath(object)
        );

        accept(lfIssue);
    }


    /**
     * Best effort to get a fileName. May return null.
     */
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


    private Path getPath(EObject object) {
        Path path;
        try {
            path = FileConfig.toPath(object.eResource());
        } catch (IOException ignored) {
            path = getFileNameBestEffort(object);
        }
        return path;
    }


    private void accept(Severity severity, String message, EObject object, int offset, int length, String code, String... issueData) {
        throw new UnsupportedOperationException("not implemented: range based diagnostics");
    }


    @Override
    public void acceptError(String message, EObject object, EStructuralFeature feature, int index, String code, String... issueData) {
        accept(Severity.ERROR, message, object, feature, index, code, issueData);
    }


    @Override
    public void acceptError(String message, EObject object, int offset, int length, String code, String... issueData) {
        accept(Severity.ERROR, message, object, offset, length, code, issueData);
    }


    @Override
    public void acceptWarning(String message, EObject object, EStructuralFeature feature, int index, String code, String... issueData) {
        accept(Severity.WARNING, message, object, feature, index, code, issueData);
    }


    @Override
    public void acceptWarning(String message, EObject object, int offset, int length, String code, String... issueData) {
        accept(Severity.WARNING, message, object, offset, length, code, issueData);
    }


    @Override
    public void acceptInfo(String message, EObject object, EStructuralFeature feature, int index, String code, String... issueData) {
        accept(Severity.INFO, message, object, feature, index, code, issueData);
    }


    @Override
    public void acceptInfo(String message, EObject object, int offset, int length, String code, String... issueData) {
        accept(Severity.INFO, message, object, offset, length, code, issueData);
    }
}
