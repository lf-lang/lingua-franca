package org.lflang;

import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.generator.Range;

/** Base implementation of the {@link MessageReporter} interface. */
public abstract class MessageReporterBase implements MessageReporter {

  private boolean errorsOccurred = false;

  protected MessageReporterBase() {}

  @Override
  public boolean getErrorsOccurred() {
    return errorsOccurred;
  }

  @Override
  public void clearHistory() {
    errorsOccurred = false;
  }

  /** A wrapper that takes care of setting the error flag if needed. */
  protected Stage2 wrap(Stage2 e) {
    return (severity, message) -> {
      if (severity == DiagnosticSeverity.Error) {
        errorsOccurred = true;
      }
      e.report(severity, message);
    };
  }

  @Override
  public Stage2 at(Path file, Range range) {
    return wrap((severity, message) -> report(file, range, severity, message));
  }

  @Override
  public Stage2 at(EObject node) {
    if (node == null) {
      return nowhere();
    }
    return wrap((severity, message) -> reportOnNode(node, severity, message));
  }

  @Override
  public Stage2 at(EObject node, EStructuralFeature feature) {
    if (node == null) {
      return nowhere();
    }
    return wrap((severity, message) -> reportOnNode(node, feature, severity, message));
  }

  @Override
  public Stage2 nowhere() {
    return wrap(this::reportWithoutPosition);
  }

  // These methods are the terminal ones that are called when a call to
  // Stage2#report is issued by a caller.

  /** Implementation of the reporting methods that use a path and range as position. */
  protected abstract void report(
      Path path, Range range, DiagnosticSeverity severity, String message);

  /** Implementation of the reporting methods that use a node as position. */
  protected void reportOnNode(EObject node, DiagnosticSeverity severity, String message) {
    reportOnNode(node, null, severity, message);
  }

  protected abstract void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message);

  /** Implementation of the reporting methods for {@link #nowhere()}. */
  protected abstract void reportWithoutPosition(DiagnosticSeverity severity, String message);
}
