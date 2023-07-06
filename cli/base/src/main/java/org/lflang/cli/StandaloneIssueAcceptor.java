package org.lflang.cli;

import com.google.inject.Inject;
import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.validation.EObjectDiagnosticImpl;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.lflang.util.FileUtil;

/** */
public class StandaloneIssueAcceptor implements ValidationMessageAcceptor {

  @Inject private IssueCollector collector;
  @Inject private ReportingBackend backend;

  boolean getErrorsOccurred() {
    return collector.getErrorsOccurred();
  }

  void accept(LfIssue lfIssue) {
    if (lfIssue.getSeverity() == Severity.INFO) {
      // print info statements instead of collecting them
      backend.printIssue(lfIssue);
    } else {
      collector.accept(lfIssue);
    }
  }

  void accept(
      Severity severity,
      String message,
      EObject object,
      EStructuralFeature feature,
      int index,
      String code,
      String... issueData) {
    EObjectDiagnosticImpl diagnostic =
        new EObjectDiagnosticImpl(severity, code, message, object, feature, index, issueData);

    LfIssue lfIssue =
        new LfIssue(
            message,
            severity,
            getPath(diagnostic),
            diagnostic.getLine(),
            diagnostic.getColumn(),
            diagnostic.getLineEnd(),
            diagnostic.getColumnEnd(),
            diagnostic.getLength());

    accept(lfIssue);
  }

  /** Best effort to get a fileName. May return null. */
  private Path getPath(EObjectDiagnosticImpl diagnostic) {
    Path file = null;
    try {
      file = FileUtil.toPath(diagnostic.getUriToProblem());
    } catch (IllegalArgumentException e) {
      // just continue with null
    }
    return file;
  }

  private void accept(
      Severity severity,
      String message,
      EObject object,
      int offset,
      int length,
      String code,
      String... issueData) {
    throw new UnsupportedOperationException("not implemented: range based diagnostics");
  }

  @Override
  public void acceptError(
      String message,
      EObject object,
      EStructuralFeature feature,
      int index,
      String code,
      String... issueData) {
    accept(Severity.ERROR, message, object, feature, index, code, issueData);
  }

  @Override
  public void acceptError(
      String message, EObject object, int offset, int length, String code, String... issueData) {
    accept(Severity.ERROR, message, object, offset, length, code, issueData);
  }

  @Override
  public void acceptWarning(
      String message,
      EObject object,
      EStructuralFeature feature,
      int index,
      String code,
      String... issueData) {
    accept(Severity.WARNING, message, object, feature, index, code, issueData);
  }

  @Override
  public void acceptWarning(
      String message, EObject object, int offset, int length, String code, String... issueData) {
    accept(Severity.WARNING, message, object, offset, length, code, issueData);
  }

  @Override
  public void acceptInfo(
      String message,
      EObject object,
      EStructuralFeature feature,
      int index,
      String code,
      String... issueData) {
    accept(Severity.INFO, message, object, feature, index, code, issueData);
  }

  @Override
  public void acceptInfo(
      String message, EObject object, int offset, int length, String code, String... issueData) {
    accept(Severity.INFO, message, object, offset, length, code, issueData);
  }
}
