package org.lflang.diagram.synthesis.util;

import de.cau.cs.kieler.klighd.Klighd;
import java.nio.file.Path;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.MessageReporterBase;
import org.lflang.generator.Range;

/**
 * Message reporter for the Lingua Franca diagram synthesis.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public class SynthesisMessageReporter extends MessageReporterBase {

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    var status =
        switch (severity) {
          case Error -> IStatus.ERROR;
          case Warning -> IStatus.WARNING;
          case Information, Hint -> IStatus.INFO;
        };
    Klighd.log(new Status(status, SynthesisMessageReporter.class, message));
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
}
