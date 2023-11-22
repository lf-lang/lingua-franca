package org.lflang.cli;

import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.JsonParser;
import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.MessageReporter;
import org.lflang.util.FileUtil;
import picocli.CommandLine;
import picocli.CommandLine.ArgGroup;
import picocli.CommandLine.Model.CommandSpec;
import picocli.CommandLine.Option;
import picocli.CommandLine.Parameters;
import picocli.CommandLine.Spec;

/**
 * Base class for standalone CLI applications.
 *
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Billy Bao
 * @author Atharva Patil
 */
public abstract class CliBase implements Runnable {
  /**
   * Models a command specification, including the options, positional parameters and subcommands
   * supported by the command.
   */
  @Spec CommandSpec spec;

  /** Options and parameters present in both Lfc and Lff. */
  static class MutuallyExclusive {
    @Parameters(
        arity = "1..",
        paramLabel = "FILES",
        description = "Paths to one or more Lingua Franca programs.")
    protected List<Path> files;

    @Option(names = "--json", description = "JSON object containing CLI arguments.")
    String jsonString;

    @Option(names = "--json-file", description = "JSON file containing CLI arguments.")
    Path jsonFile;

    @Option(names = "--stdin", description = "Read paths to Lingua Franca programs from stdin.")
    private boolean stdin;
  }

  @ArgGroup(multiplicity = "1")
  MutuallyExclusive topLevelArg;

  @Option(
      names = {"-o", "--output-path"},
      defaultValue = "",
      description = "Specify the root output directory.")
  private Path outputPath;

  /** Used to collect all errors that happen during validation/generation. */
  @Inject protected IssueCollector issueCollector;

  /** Used to report error messages at the end. */
  @Inject protected ReportingBackend reporter;

  /** Used to report error messages at the end. */
  @Inject protected MessageReporter messageReporter;

  /** IO context of this run. */
  @Inject protected Io io;

  /** Injected resource provider. */
  @Inject private Provider<ResourceSet> resourceSetProvider;

  /** Injected resource validator. */
  @Inject private IResourceValidator validator;

  private JsonObject jsonObject;

  protected static void cliMain(
      String toolName, Class<? extends CliBase> toolClass, Io io, String[] args) {
    // Injector used to obtain Main instance.
    final Injector injector = getInjector(toolName, io);
    // Main instance.
    final CliBase main = injector.getInstance(toolClass);
    // Parse arguments and execute main logic.
    main.doExecute(io, args);
  }

  public void doExecute(Io io, String[] args) {
    CommandLine cmd =
        new CommandLine(this)
            .setOut(new PrintWriter(io.getOut()))
            .setErr(new PrintWriter(io.getErr()));
    int exitCode = cmd.execute(args);
    io.callSystemExit(exitCode);
  }

  /**
   * The entrypoint of Picocli applications - the first method called when CliBase, which implements
   * the Runnable interface, is instantiated.
   */
  public void run() {
    doRun();
  }

  /*
   * The entrypoint of tool-specific logic.
   * Lfc and Lff have their own specific implementations for this method.
   */
  public abstract void doRun();

  public static Injector getInjector(String toolName, Io io) {
    final ReportingBackend reporter = new ReportingBackend(io, toolName + ": ");

    // Injector used to obtain Main instance.
    return new LFStandaloneSetup(new LFRuntimeModule(), new LFStandaloneModule(reporter, io))
        .createInjectorAndDoEMFRegistration();
  }

  /** Resolve to an absolute path, in the given {@link #io} context. */
  protected Path toAbsolutePath(Path other) {
    return io.getWd().resolve(other).toAbsolutePath();
  }

  /**
   * Returns the validated input paths.
   *
   * @return Validated input paths.
   */
  protected List<Path> getInputPaths() {
    List<Path> paths;
    if (topLevelArg.stdin) {
      var input = new BufferedInputStream(System.in);
      var reader = new BufferedReader(new InputStreamReader(input, StandardCharsets.UTF_8));
      String line;
      try {
        line = reader.readLine();
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
      if (line == null) return List.of();
      return List.of(Path.of(line));
    } else if (topLevelArg.jsonFile != null || topLevelArg.jsonString != null) {
      paths = new ArrayList<>();
      var filesObj = getJsonObject().get("src");
      if (filesObj != null) {
        if (filesObj.isJsonPrimitive()) {
          paths = List.of(Path.of(filesObj.getAsString()));
        } else if (filesObj.isJsonArray()) {
          paths =
              filesObj.getAsJsonArray().asList().stream()
                  .map(e -> Path.of(e.getAsString()))
                  .toList();
        } else {
          reporter.printFatalErrorAndExit(
              "JSON Parse Exception: field \"src\" must be a string or an array of strings.");
        }
      } else {
        reporter.printFatalErrorAndExit("JSON Parse Exception: field \"src\" not found.");
      }
    } else {
      paths = topLevelArg.files.stream().map(io.getWd()::resolve).collect(Collectors.toList());
    }

    for (Path path : paths) {
      if (!Files.exists(path)) {
        reporter.printFatalErrorAndExit(path + ": No such file or directory.");
      }
    }

    return paths;
  }

  protected final JsonObject getJsonObject() {
    if (jsonObject != null) {
      return jsonObject;
    }
    var jsonString = topLevelArg.jsonString;
    // If args are given in a json file, store its contents in jsonString.
    if (topLevelArg.jsonFile != null) {
      try {
        jsonString = new String(Files.readAllBytes(io.getWd().resolve(topLevelArg.jsonFile)));
      } catch (IOException e) {
        reporter.printFatalErrorAndExit("No such file: " + topLevelArg.jsonFile);
      }
    }
    if (jsonString != null) {
      try {
        jsonObject = JsonParser.parseString(jsonString).getAsJsonObject();
      } catch (JsonParseException e) {
        messageReporter.nowhere().error(String.format("Invalid JSON string:%n %s", jsonString));
      }
    }
    return jsonObject;
  }

  protected final boolean stdinMode() {
    return topLevelArg.stdin;
  }

  /**
   * Returns the validated, normalized output path.
   *
   * @return Validated, normalized output path.
   */
  protected Path getOutputRoot() {
    Path root = null;
    Path path = null;

    if (!outputPath.toString().isEmpty()) {
      path = outputPath;
    } else {
      var json = getJsonObject();
      if (json != null) {
        var obj = json.get("out");
        if (obj != null) {
          path = Path.of(obj.getAsString());
        }
      }
    }

    if (path != null) {
      root = io.getWd().resolve(path).normalize();
      if (!Files.exists(root)) {
        reporter.printFatalErrorAndExit(root + ": Output location does not exist.");
      }
      if (!Files.isDirectory(root)) {
        reporter.printFatalErrorAndExit(root + ": Output location is not a directory.");
      }
    }
    return root;
  }

  /** If some errors were collected, print them and abort execution. Otherwise, return. */
  protected void exitIfCollectedErrors() {
    if (issueCollector.getErrorsOccurred()) {

      // Print warnings if there are any.
      printWarningsIfAny();

      List<LfIssue> errors = printErrorsIfAny();
      String cause = errors.size() + " previous error";
      if (errors.size() > 1) {
        cause += 's';
      }
      reporter.printFatalErrorAndExit("Aborting due to " + cause + '.');
    }
  }

  /**
   * If any warnings were collected, print them, then return them.
   *
   * @return A list of collected warnings.
   */
  public List<LfIssue> printWarningsIfAny() {
    List<LfIssue> errors = issueCollector.getWarnings();
    errors.forEach(reporter::printIssue);
    return errors;
  }

  /**
   * If any warnings were collected, print them, then return them.
   *
   * @return A list of collected errors.
   */
  public List<LfIssue> printErrorsIfAny() {
    List<LfIssue> errors = issueCollector.getErrors();
    errors.forEach(reporter::printIssue);
    return errors;
  }

  /**
   * Validates a given resource. If issues arise during validation, these are recorded using the
   * issue collector.
   *
   * @param resource The resource to validate.
   */
  public void validateResource(Resource resource) {
    assert resource != null;

    List<Issue> issues = this.validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl);

    for (Issue issue : issues) {
      // Issues may also relate to imported resources.
      URI uri = issue.getUriToProblem();
      Path path = null;
      if (uri != null) {
        try {
          path = FileUtil.toPath(uri);
        } catch (IllegalArgumentException e) {
          reporter.printError("Unable to convert '" + uri + "' to path. " + e);
        }
      }
      issueCollector.accept(
          new LfIssue(
              issue.getMessage(),
              issue.getSeverity(),
              path,
              issue.getLineNumber(),
              issue.getColumn(),
              issue.getLineNumberEnd(),
              issue.getColumnEnd(),
              issue.getLength()));
    }
  }

  /**
   * Obtains a resource from a path. Returns null if path is not an LF file.
   *
   * @param path The path to obtain the resource from.
   * @return The obtained resource. Set to null if path is not an LF file.
   */
  public Resource getResource(Path path) {
    final ResourceSet set = this.resourceSetProvider.get();
    try {
      return set.getResource(URI.createFileURI(path.toString()), true);
    } catch (RuntimeException e) {
      return null;
    }
  }
}
