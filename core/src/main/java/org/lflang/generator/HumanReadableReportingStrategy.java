package org.lflang.generator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Iterator;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.MessageReporter;

/**
 * An error reporting strategy that parses human-readable output.
 *
 * @author Peter Donovan
 */
public class HumanReadableReportingStrategy implements DiagnosticReporting.Strategy {

  /** A pattern that matches lines that should be reported via this strategy. */
  private final Pattern diagnosticMessagePattern;

  /** A pattern that matches labels that show the exact range to which the diagnostic pertains. */
  private final Pattern labelPattern;

  /** The path against which any paths should be resolved. */
  private final Path relativeTo;

  /** The next line to be processed, or {@code null}. */
  private String bufferedLine;

  /**
   * Instantiate a reporting strategy for lines of validator output that match {@code
   * diagnosticMessagePattern}.
   *
   * @param diagnosticMessagePattern A pattern that matches lines that should be reported via this
   *     strategy. This pattern must contain named capturing groups called "path", "line", "column",
   *     "message", and "severity".
   * @param labelPattern A pattern that matches lines that act as labels, showing the location of
   *     the relevant piece of text. This pattern must contain two groups, the first of which must
   *     match characters that precede the location given by the "line" and "column" groups.
   */
  public HumanReadableReportingStrategy(Pattern diagnosticMessagePattern, Pattern labelPattern) {
    this(diagnosticMessagePattern, labelPattern, null);
  }

  /**
   * Instantiate a reporting strategy for lines of validator output that match {@code
   * diagnosticMessagePattern}.
   *
   * @param diagnosticMessagePattern a pattern that matches lines that should be reported via this
   *     strategy. This pattern must contain named capturing groups called "path", "line", "column",
   *     "message", and "severity".
   * @param labelPattern A pattern that matches lines that act as labels, showing the location of
   *     the relevant piece of text. This pattern must contain two groups, the first of which must
   *     match characters that precede the location given by the "line" and "column" groups.
   * @param relativeTo The path against which any paths should be resolved.
   */
  public HumanReadableReportingStrategy(
      Pattern diagnosticMessagePattern, Pattern labelPattern, Path relativeTo) {
    for (String groupName : new String[] {"path", "line", "column", "message", "severity"}) {
      assert diagnosticMessagePattern.pattern().contains(groupName)
          : String.format(
              "Error line patterns must have a named capturing group called %s", groupName);
    }
    this.diagnosticMessagePattern = diagnosticMessagePattern;
    this.labelPattern = labelPattern;
    this.relativeTo = relativeTo;
    this.bufferedLine = null;
  }

  @Override
  public void report(
      String validationOutput, MessageReporter messageReporter, Map<Path, CodeMap> map) {
    Iterator<String> it = validationOutput.lines().iterator();
    while (it.hasNext() || bufferedLine != null) {
      if (bufferedLine != null) {
        reportErrorLine(bufferedLine, it, messageReporter, map);
        bufferedLine = null;
      } else {
        reportErrorLine(it.next(), it, messageReporter, map);
      }
    }
  }

  /**
   * Report the validation message contained in the given line of text.
   *
   * @param line The current line.
   * @param it An iterator over the lines that follow the current line.
   * @param messageReporter An arbitrary ErrorReporter.
   * @param maps A mapping from generated file paths to CodeMaps.
   */
  private void reportErrorLine(
      String line, Iterator<String> it, MessageReporter messageReporter, Map<Path, CodeMap> maps) {
    Matcher matcher = diagnosticMessagePattern.matcher(stripEscaped(line));
    if (matcher.matches()) {
      final Path path = Paths.get(matcher.group("path"));
      final DiagnosticSeverity severity = DiagnosticReporting.severityOf(matcher.group("severity"));

      String column = matcher.group("column");
      final Position generatedFilePosition =
          Position.fromOneBased(
              Integer.parseInt(matcher.group("line")),
              column == null
                  ? 1 // FIXME: Unreliable heuristic
                  : Integer.parseInt(column));
      final String message =
          DiagnosticReporting.messageOf(matcher.group("message"), path, generatedFilePosition);
      final CodeMap map = maps.get(relativeTo != null ? relativeTo.resolve(path) : path);
      if (map == null) {
        messageReporter.nowhere().report(severity, message);
        return;
      }
      for (Path srcFile : map.lfSourcePaths()) {
        Position lfFilePosition = map.adjusted(srcFile, generatedFilePosition);
        if (column != null) {
          Range range = findAppropriateRange(lfFilePosition, it);
          messageReporter.at(srcFile, range).report(severity, message);
        } else {
          messageReporter.at(srcFile, lfFilePosition.getOneBasedLine()).report(severity, message);
        }
      }
    }
  }

  /**
   * Find the appropriate range to {@code report}.
   *
   * @param lfFilePosition The point about which the relevant range is anchored.
   * @param it An iterator over the lines immediately following a diagnostic message.
   */
  private Range findAppropriateRange(Position lfFilePosition, Iterator<String> it) {
    while (it.hasNext()) {
      String line = it.next();
      Matcher labelMatcher = labelPattern.matcher(line);
      if (labelMatcher.find()) {
        Position start =
            Position.fromZeroBased(
                lfFilePosition.getZeroBasedLine(),
                lfFilePosition.getZeroBasedColumn() - labelMatcher.group(1).length());
        Position end = lfFilePosition.plus(labelMatcher.group(2));
        return new Range(start, end);
      } else if (diagnosticMessagePattern.matcher(line).find()) {
        bufferedLine = line;
        break;
      }
    }
    // fallback, we didn't find it.
    return new Range(lfFilePosition, lfFilePosition.plus(" "));
  }

  /**
   * Strip the ANSI escape sequences from {@code s}.
   *
   * @param s Any string.
   * @return {@code s}, with any escape sequences removed.
   */
  private static String stripEscaped(String s) {
    return s.replaceAll("\u001B\\[[;\\d]*[ -/]*[@-~]", "");
  }
}
