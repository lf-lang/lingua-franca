package org.lflang.generator;

import java.nio.file.Path;
import java.util.Map;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.MessageReporter;

/**
 * {@code DiagnosticReporting} provides utilities for reporting validation output.
 *
 * @author Peter Donovan
 */
public class DiagnosticReporting {

  private DiagnosticReporting() {
    // utility class
  }

  /** A means of parsing the output of a validator. */
  @FunctionalInterface
  public interface Strategy {
    /**
     * Parse the validation output and report any errors that it contains.
     *
     * @param validationOutput any validation output
     * @param messageReporter any error reporter
     * @param map the map from generated files to CodeMaps
     */
    void report(String validationOutput, MessageReporter messageReporter, Map<Path, CodeMap> map);
  }

  /**
   * Format the given data as a human-readable message.
   *
   * @param message An error message.
   * @param path The path of the source of the message.
   * @param position The position where the message originates.
   * @return The given data as a human-readable message.
   */
  public static String messageOf(String message, Path path, Position position) {
    return String.format(
        "%s [%s:%s:%s]",
        message,
        path.getFileName().toString(),
        position.getOneBasedLine(),
        position.getOneBasedColumn());
  }

  /**
   * Convert {@code severity} into a {@code DiagnosticSeverity} using a heuristic that should be
   * compatible with many tools.
   *
   * @param severity The string representation of a diagnostic severity.
   * @return The {@code DiagnosticSeverity} representation of {@code severity}.
   */
  public static DiagnosticSeverity severityOf(String severity) {
    severity = severity.toLowerCase();
    if (severity.contains("error")) return DiagnosticSeverity.Error;
    else if (severity.contains("warning")) return DiagnosticSeverity.Warning;
    else if (severity.contains("hint") || severity.contains("help")) return DiagnosticSeverity.Hint;
    else return DiagnosticSeverity.Information;
  }
}
