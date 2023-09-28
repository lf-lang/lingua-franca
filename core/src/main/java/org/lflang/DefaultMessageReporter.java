package org.lflang;

import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.generator.Range;

/** Simple implementation of the ErrorReport interface that simply prints to standard out. */
public class DefaultMessageReporter extends MessageReporterBase implements MessageReporter {

  private void println(String s) {
    System.out.println(s);
  }

  @Override
  protected void report(Path path, Range range, DiagnosticSeverity severity, String message) {
    reportWithoutPosition(severity, message);
  }

  @Override
  protected void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message) {
    reportWithoutPosition(severity, message);
  }

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    switch (severity) {
      case Error -> println("ERROR: " + message);
      case Warning -> println("WARNING: " + message);
      case Information, Hint -> println("INFO: " + message);
    }
  }
}
