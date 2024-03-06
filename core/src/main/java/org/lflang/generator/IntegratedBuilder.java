package org.lflang.generator;

import com.google.gson.JsonParser;
import com.google.inject.Inject;
import com.google.inject.Provider;
import java.nio.file.Path;
import java.util.List;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.generator.LFGeneratorContext.Mode;

/**
 * Manages Lingua Franca build processes that are requested from the language server.
 *
 * @author Peter Donovan
 */
public class IntegratedBuilder {
  public static final int START_PERCENT_PROGRESS = 0;
  public static final int VALIDATED_PERCENT_PROGRESS = 33;
  public static final int GENERATED_PERCENT_PROGRESS = 67;
  public static final int COMPILED_PERCENT_PROGRESS = 100;

  /** A {@code ProgressReporter} reports the progress of a build. */
  public interface ReportProgress {
    void apply(String message, Integer percentage);
  }

  /* ---------------------- INJECTED DEPENDENCIES ---------------------- */

  @Inject private IResourceValidator validator;
  @Inject private GeneratorDelegate generator;
  @Inject private JavaIoFileSystemAccess fileAccess;
  @Inject private Provider<ResourceSet> resourceSetProvider;

  /* ------------------------- PUBLIC METHODS -------------------------- */

  /**
   * Generates code from the Lingua Franca file {@code f}.
   *
   * @param uri The URI of a Lingua Franca file.
   * @param mustComplete Whether the build must be taken to completion.
   * @return The result of the build.
   */
  public GeneratorResult run(
      URI uri,
      String json,
      boolean mustComplete,
      ReportProgress reportProgress,
      CancelIndicator cancelIndicator) {
    fileAccess.setOutputPath(
        FileConfig.findPackageRoot(Path.of(uri.path()), s -> {})
            .resolve(FileConfig.DEFAULT_SRC_GEN_DIR)
            .toString());
    List<EObject> parseRoots = getResource(uri).getContents();
    if (parseRoots.isEmpty()) return GeneratorResult.NOTHING;
    MessageReporter messageReporter = new LanguageServerMessageReporter(parseRoots.get(0));
    reportProgress.apply("Validating...", START_PERCENT_PROGRESS);
    validate(uri, messageReporter);
    reportProgress.apply("Code validation complete.", VALIDATED_PERCENT_PROGRESS);
    if (cancelIndicator.isCanceled()) return GeneratorResult.CANCELLED;
    if (messageReporter.getErrorsOccurred()) return GeneratorResult.FAILED;
    reportProgress.apply("Generating code...", VALIDATED_PERCENT_PROGRESS);
    return doGenerate(uri, json, mustComplete, reportProgress, cancelIndicator);
  }

  /* ------------------------- PRIVATE METHODS ------------------------- */

  /**
   * Validates the Lingua Franca file {@code f}.
   *
   * @param uri The URI of a Lingua Franca file.
   * @param messageReporter The error reporter.
   */
  private void validate(URI uri, MessageReporter messageReporter) {
    for (Issue issue :
        validator.validate(getResource(uri), CheckMode.ALL, CancelIndicator.NullImpl)) {
      messageReporter
          .atNullableLine(Path.of(uri.path()), issue.getLineNumber())
          .report(convertSeverity(issue.getSeverity()), issue.getMessage());
    }
  }

  /**
   * Generates code from the contents of {@code f}.
   *
   * @param uri The URI of a Lingua Franca file.
   * @param mustComplete Whether the build must be taken to completion.
   * @param cancelIndicator An indicator that returns true when the build is cancelled.
   * @return The result of the build.
   */
  private GeneratorResult doGenerate(
      URI uri,
      String json,
      boolean mustComplete,
      ReportProgress reportProgress,
      CancelIndicator cancelIndicator) {
    var resource = getResource(uri);
    LFGeneratorContext context =
        new MainContext(
            mustComplete ? Mode.LSP_SLOW : LFGeneratorContext.Mode.LSP_MEDIUM,
            cancelIndicator,
            reportProgress,
            getArgs(json),
            resource,
            fileAccess,
            fileConfig -> new LanguageServerMessageReporter(resource.getContents().get(0)));
    generator.generate(getResource(uri), fileAccess, context);
    return context.getResult();
  }

  /**
   * Returns the resource corresponding to {@code uri}.
   *
   * @param uri The URI of a Lingua Franca file.
   * @return The resource corresponding to {@code uri}.
   */
  public Resource getResource(URI uri) {
    return resourceSetProvider.get().getResource(uri, true);
  }

  static DiagnosticSeverity convertSeverity(Severity severity) {
    return switch (severity) {
      case ERROR -> DiagnosticSeverity.Error;
      case WARNING -> DiagnosticSeverity.Warning;
      case INFO -> DiagnosticSeverity.Information;
      case IGNORE -> DiagnosticSeverity.Hint;
    };
  }

  /** Return arguments to feed to the code generator. Currently, no arguments are being set. */
  protected GeneratorArguments getArgs(String jsonString) {
    if (jsonString == null || jsonString.isBlank()) {
      return GeneratorArguments.none();
    }
    var json = JsonParser.parseString(jsonString).getAsJsonObject();
    return new GeneratorArguments(false, null, false, json, false, false, null, List.of());
  }
}
