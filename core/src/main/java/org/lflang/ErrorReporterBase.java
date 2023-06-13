package org.lflang;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.generator.Range;

/** Simple implementation of the ErrorReport interface that simply prints to standard out. */
public abstract class ErrorReporterBase implements ErrorReporter {

  private boolean errorsOccurred = false;

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
  public Stage2 at(EObject object) {
    return wrap((severity, message) -> reportOnNode(object, severity, message));
  }

  @Override
  public Stage2 nowhere() {
    return wrap(this::reportWithoutPosition);
  }

  protected abstract void report(Path path, Range range, DiagnosticSeverity severity, String message);

  protected abstract void reportOnNode(EObject node, DiagnosticSeverity severity, String message);

  protected abstract void reportWithoutPosition(DiagnosticSeverity severity, String message);

}
