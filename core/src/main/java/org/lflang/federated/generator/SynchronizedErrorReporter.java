package org.lflang.federated.generator;

import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.ErrorReporter;
import org.lflang.ErrorReporterBase;
import org.lflang.generator.Position;
import org.lflang.generator.Range;

public class SynchronizedErrorReporter extends ErrorReporterBase {

  private final ErrorReporter parent;

  public SynchronizedErrorReporter(ErrorReporter parent) {
    this.parent = parent;
  }

  @Override
  protected synchronized void reportOnNode(EObject node, DiagnosticSeverity severity, String message) {
    parent.at(node).report(severity, message);
  }

  @Override
  protected synchronized void report(Path path, Range range, DiagnosticSeverity severity, String message) {
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
