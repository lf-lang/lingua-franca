package org.lflang.cli;

import com.google.inject.Inject;
import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.diagnostics.Severity;
import org.lflang.MessageReporterBase;
import org.lflang.generator.Range;

/**
 * An error reporter that forwards all messages to an `IssueCollector`. They'll be sorted out
 * later.
 *
 * @ingroup CLI
 */
public class StandaloneMessageReporter extends MessageReporterBase {

  @Inject private StandaloneIssueAcceptor issueAcceptor;

  static Severity convertSeverity(DiagnosticSeverity severity) {
    return switch (severity) {
      case Error -> Severity.ERROR;
      case Warning -> Severity.WARNING;
      case Information, Hint -> Severity.INFO;
    };
  }

  @Override
  protected void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message) {
    issueAcceptor.accept(convertSeverity(severity), message, node, feature, 0, null);
  }

  @Override
  protected void report(Path path, Range range, DiagnosticSeverity severity, String message) {
    LfIssue issue = new LfIssue(message, convertSeverity(severity), path, range);
    issueAcceptor.accept(issue);
  }

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    LfIssue issue = new LfIssue(message, convertSeverity(severity), null, null);
    issueAcceptor.accept(issue);
  }

  @Override
  public boolean getErrorsOccurred() {
    return issueAcceptor.getErrorsOccurred();
  }
}
