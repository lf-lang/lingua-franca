package org.lflang.generator;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.lsp4j.PublishDiagnosticsParams;
import org.eclipse.lsp4j.Range;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.MessageReporterBase;
import org.lflang.util.FileUtil;

/**
 * Report diagnostics to the language client.
 *
 * @author Peter Donovan
 */
public class LanguageServerMessageReporter extends MessageReporterBase {

  /**
   * The language client to which errors should be reported, if such a client is available. FIXME:
   * This is a de facto global, and it is a hack.
   */
  private static LanguageClient client;

  /** The document for which this is a diagnostic acceptor. */
  private final EObject parseRoot;

  /** The list of all diagnostics since the last reset. */
  private final Map<Path, List<Diagnostic>> diagnostics;

  /**
   * Initialize a {@code DiagnosticAcceptor} for the document whose parse tree root node is {@code
   * parseRoot}.
   *
   * @param parseRoot the root of the AST of the document for which this is an error reporter
   */
  public LanguageServerMessageReporter(EObject parseRoot) {
    this.parseRoot = parseRoot;
    this.diagnostics = new HashMap<>();
  }

  @Override
  protected void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message) {
    reportWithoutPosition(severity, message);
  }

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    report(
        getMainFile(),
        org.lflang.generator.Range.degenerateRange(Position.ORIGIN),
        severity,
        message);
  }

  @Override
  public Stage2 at(Path file, int line) {
    // Create a range for the whole line
    Optional<String> text = getLine(line - 1);
    org.lflang.generator.Range range =
        new org.lflang.generator.Range(
            Position.fromOneBased(line, 1),
            Position.fromOneBased(line, 1 + text.map(String::length).orElse(0)));
    return at(file, range);
  }

  @Override
  protected void report(
      Path path, org.lflang.generator.Range range, DiagnosticSeverity severity, String message) {
    if (path == null) {
      path = getMainFile();
    }
    diagnostics
        .computeIfAbsent(path, p -> new ArrayList<>())
        .add(
            new Diagnostic(
                toRange(range.getStartInclusive(), range.getEndExclusive()),
                message,
                severity,
                "LF Language Server"));
  }

  @Override
  public boolean getErrorsOccurred() {
    return diagnostics.values().stream()
        .anyMatch(
            it ->
                it.stream()
                    .anyMatch(diagnostic -> diagnostic.getSeverity() == DiagnosticSeverity.Error));
  }

  /**
   * Save a reference to the language client.
   *
   * @param client the language client
   */
  public static void setClient(LanguageClient client) {
    LanguageServerMessageReporter.client = client;
  }

  /** Publish diagnostics by forwarding them to the language client. */
  public void publishDiagnostics() {
    if (client == null) {
      System.err.println(
          "WARNING: Cannot publish diagnostics because the language client has not yet been"
              + " found.");
      return;
    }
    for (Path file : diagnostics.keySet()) {
      PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
      publishDiagnosticsParams.setUri(URI.createFileURI(file.toString()).toString());
      publishDiagnosticsParams.setDiagnostics(diagnostics.get(file));
      client.publishDiagnostics(publishDiagnosticsParams);
    }
  }

  /* -----------------------  PRIVATE METHODS  ------------------------ */

  /** Return the file on which the current validation process was triggered. */
  private Path getMainFile() {
    return FileUtil.toPath(parseRoot.eResource().getURI());
  }

  /**
   * Return the text of the document for which this is an error reporter.
   *
   * @return the text of the document for which this is an error reporter
   */
  private String getText() {
    return NodeModelUtils.getNode(parseRoot).getText();
  }

  /**
   * Return the line at index {@code line} in the document for which this is an error reporter.
   *
   * @param line the zero-based line index
   * @return the line located at the given index
   */
  private Optional<String> getLine(int line) {
    return getText().lines().skip(line).findFirst();
  }

  /**
   * Return the Range that starts at {@code p0} and ends at {@code p1}.
   *
   * @param p0 an arbitrary Position
   * @param p1 a Position that is greater than {@code p0}
   * @return the Range that starts at {@code p0} and ends at {@code p1}
   */
  private Range toRange(Position p0, Position p1) {
    return new Range(
        new org.eclipse.lsp4j.Position(p0.getZeroBasedLine(), p0.getZeroBasedColumn()),
        new org.eclipse.lsp4j.Position(p1.getZeroBasedLine(), p1.getZeroBasedColumn()));
  }
}
