package org.lflang;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.Stage2;
import org.lflang.generator.Position;
import org.lflang.generator.Range;

/**
 * Interface for reporting errors.
 *
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Christian Menard
 */
public interface ErrorReporter {

  Stage2 at(Path file, Range range);

  Stage2 at(EObject object);

  default Stage2 at(Path file) {
    return at(file, 1);
  }

  default Stage2 at(Path file, int line) {
    return at(file, Position.fromOneBased(line, 1));
  }

  default Stage2 at(Path file, Position pos) {
    return at(file, Range.degenerateRange(pos));
  }

  Stage2 nowhere();


  interface Stage2 {

    /**
     * Report an error.
     *
     * @param message The error message.
     */
    default void error(String message) {
      report(DiagnosticSeverity.Error, message);
    }

    /**
     * Report a warning.
     *
     * @param message The warning message.
     */
    default void warning(String message) {
      report(DiagnosticSeverity.Warning, message);
    }

    /**
     * Report an informational message.
     *
     * @param message The message to report
     */
    default void info(String message) {
      report(DiagnosticSeverity.Information, message);
    }


    /**
     * Report a message with the given severity
     *
     * @param severity The severity
     * @param message The message to report
     */
    void report(DiagnosticSeverity severity, String message);

  }


  /**
   * Report an error at the specified line within a file.
   *
   * @param message The error message.
   * @param line The one-based line number to report at.
   * @param file The file to report at.
   * @return a string that describes the error.
   */
  default String reportError(Path file, Integer line, String message) {
    Stage2 stage2 = line != null ? at(file, line) : at(file);
    stage2.report(DiagnosticSeverity.Error, message);
    return message;
  }

  /**
   * Report a warning at the specified line within a file.
   *
   * @param message The error message.
   * @param line The one-based line number to report at.
   * @param file The file to report at.
   * @return a string that describes the warning.
   */
  default String reportWarning(Path file, Integer line, String message) {
    Stage2 stage2 = line!=null? at(file,line):at(file);
    stage2.report(DiagnosticSeverity.Warning, message);
    return message;
  }


  /**
   * Report a message of severity {@code severity}.
   *
   * @param file The file to which the message pertains, or {@code null} if the file is unknown.
   * @param severity the severity of the message
   * @param message the message to send to the IDE
   * @return a string that describes the diagnostic
   */
  default String report(Path file, DiagnosticSeverity severity, String message) {
    at(file).report(severity, message);
    return message;
  }

  /**
   * Report a message of severity {@code severity} that pertains to line {@code line} of an LF
   * source file.
   *
   * @param file The file to which the message pertains, or {@code null} if the file is unknown.
   * @param severity the severity of the message
   * @param message the message to send to the IDE
   * @param line the one-based line number associated with the message
   * @return a string that describes the diagnostic
   */
  default String report(Path file, DiagnosticSeverity severity, String message, int line) {
    at(file, line).report(severity, message);
    return message;
  }

  /**
   * Report a message of severity {@code severity} that pertains to the range [{@code startPos},
   * {@code endPos}) of an LF source file.
   *
   * @param file The file to which the message pertains, or {@code null} if the file is unknown.
   * @param severity the severity of the message
   * @param message the message to send to the IDE
   * @param startPos the position of the first character of the range of interest
   * @param endPos the position immediately AFTER the final character of the range of interest
   * @return a string that describes the diagnostic
   */
  default String report(
      Path file, DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
    return report(file, severity, message, startPos.getOneBasedLine());
  }


  /**
   * Check if errors where reported.
   *
   * @return true if errors where reported
   */
  boolean getErrorsOccurred();

  /**
   * Clear error history, if exists. This is usually only the case for error markers in Epoch
   * (Eclipse).
   */
  default void clearHistory() {}
}
