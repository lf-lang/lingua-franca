package org.lflang.federated.generator;

import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.MessageReporter;
import org.lflang.MessageReporterBase;
import org.lflang.generator.Range;

public class SynchronizedMessageReporter extends MessageReporterBase {

  private final MessageReporter parent;

  public SynchronizedMessageReporter(MessageReporter parent) {
    this.parent = parent;
  }

  @Override
  protected synchronized void reportOnNode(
      EObject node, DiagnosticSeverity severity, String message) {
    parent.at(node).report(severity, message);
  }

  @Override
  protected synchronized void report(
      Path path, Range range, DiagnosticSeverity severity, String message) {
    parent.at(path, range).report(severity, message);
  }

  @Override
  protected synchronized void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    parent.nowhere().report(severity, message);
  }

  @Override
  public synchronized boolean getErrorsOccurred() {
    return parent.getErrorsOccurred();
  }
}
