package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.Map;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.ErrorReporter;
import org.lflang.ErrorReporterBase;
import org.lflang.generator.CodeMap;
import org.lflang.generator.Position;
import org.lflang.generator.Range;

public class LineAdjustingErrorReporter implements ErrorReporter {

  private final ErrorReporter parent;
  private final Map<Path, CodeMap> codeMapMap;

  public LineAdjustingErrorReporter(ErrorReporter parent, Map<Path, CodeMap> codeMapMap) {
    this.parent = parent;
    this.codeMapMap = codeMapMap;
  }


  @Override
  public Stage2 at(EObject object) {
    return parent.at(object);
  }

  @Override
  public Stage2 nowhere() {
    return parent.nowhere();
  }

  @Override
  public Stage2 at(Path file, int line) {
    // encompass the whole line
    var endOfLine = Position.fromOneBased(line, Integer.MAX_VALUE);
    var startOfLine = Position.fromOneBased(line, 1);
    return at(file, new Range(startOfLine, endOfLine));
  }

  @Override
  public Stage2 at(Path file, Range range) {
    if (codeMapMap.containsKey(file)) {
      var relevantMap = codeMapMap.get(file);
      for (Path lfSource : relevantMap.lfSourcePaths()) {
        var adjustedRange = relevantMap.adjusted(lfSource, range);
        adjustedRange = new Range(
            adjustedRange.getStartInclusive().equals(Position.ORIGIN)
                ? Position.fromZeroBased(adjustedRange.getEndExclusive().getZeroBasedLine(), 0)
                : adjustedRange.getStartInclusive(),
            adjustedRange.getEndExclusive());
        return parent.at(lfSource, adjustedRange);
      }
    }
    return nowhere();
  }


  @Override
  public boolean getErrorsOccurred() {
    return parent.getErrorsOccurred();
  }
}
