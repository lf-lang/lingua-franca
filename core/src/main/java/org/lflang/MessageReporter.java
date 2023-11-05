package org.lflang;

import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.generator.Position;
import org.lflang.generator.Range;

/**
 * Interface for reporting messages like errors or info. This interface is a staged builder: first
 * call one of the {@code at} methods to specify the position of the message, then use one of the
 * report methods on the returned {@link Stage2} instance.
 *
 * <p>Examples:
 *
 * <pre>{@code
 * errorReporter.at(file, line).error("an error")
 * errorReporter.at(node).warning("a warning reported on a node")
 * errorReporter.nowhere().error("Some error that has no specific position")
 * }</pre>
 *
 * @see MessageReporterBase
 * @author Edward A. Lee
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Clément Fournier
 */
public interface MessageReporter {

  /** Position the message on the given range in a given file (both must be non-null). */
  Stage2 at(Path file, Range range);

  /** Position the message on the given node (must be non-null). */
  Stage2 at(EObject node);

  /** Position the message on the given node and structural feature (both must be non-null). */
  Stage2 at(EObject node, EStructuralFeature feature);

  /**
   * Position the message in the file (non-null), at an unknown line. Implementations usually will
   * report on the first line of the file.
   */
  default Stage2 at(Path file) {
    return at(file, 1);
  }

  /** Position the message in the file (non-null), on the given line. */
  default Stage2 at(Path file, int line) {
    return at(file, Position.fromOneBased(line, 1));
  }

  /** Position the message in the file, using a position object. */
  default Stage2 at(Path file, Position pos) {
    return at(file, Range.degenerateRange(pos));
  }

  /**
   * Specify that the message has no relevant position, ie it does not belong to a particular file.
   */
  Stage2 nowhere();

  /**
   * Position the message in the given file. The line may be null. This is a convenience wrapper
   * that calls either {@link #at(Path, int)} or {@link #at(Path)}.
   */
  default Stage2 atNullableLine(Path file, Integer line) {
    if (line != null) {
      return at(file, line);
    }
    return at(file);
  }

  /**
   * Interface to report a message with a specific severity. This is returned by one of the
   * positioning functions like {@link #at(Path)}. This instance holds an implicit position.
   */
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
     * Report a message with the given severity. This is the only member that needs to be
     * implemented.
     *
     * @param severity The severity
     * @param message The message to report
     */
    void report(DiagnosticSeverity severity, String message);
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
