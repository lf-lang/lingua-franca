package org.lflang.cli;

import com.google.gson.JsonElement;
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
import java.util.Map.Entry;
import java.util.Set;
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
    private String jsonString;

    @Option(names = "--json-file", description = "JSON file containing CLI arguments.")
    private Path jsonFile;

    @Option(names = "--stdin", description = "Read paths to Lingua Franca programs from stdin.")
    private boolean stdin;
  }

  @ArgGroup(exclusive = true, multiplicity = "1")
  MutuallyExclusive topLevelArg;

  @Option(
      names = {"-o", "--output-path"},
      defaultValue = "",
      fallbackValue = "",
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
    // If args are given in a json file, store its contents in jsonString.
    if (topLevelArg.jsonFile != null) {
      try {
        topLevelArg.jsonString =
            new String(Files.readAllBytes(io.getWd().resolve(topLevelArg.jsonFile)));
      } catch (IOException e) {
        reporter.printFatalErrorAndExit("No such file: " + topLevelArg.jsonFile);
      }
    }
    // If args are given in a json string, unpack them and re-run
    // picocli argument validation.
    if (topLevelArg.jsonString != null) {
      // Unpack args from json string.
      String[] args = jsonStringToArgs(topLevelArg.jsonString);
      // Execute application on unpacked args.
      CommandLine cmd = spec.commandLine();
      cmd.execute(args);
      // If args are already unpacked, invoke tool-specific logic.
    } else {
      doRun();
    }
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
    if (!outputPath.toString().isEmpty()) {
      root = io.getWd().resolve(outputPath).normalize();
      if (!Files.exists(root)) { // FIXME: Create it instead?
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
      // if there are errors, don't print warnings.
      List<LfIssue> errors = printErrorsIfAny();
      String cause = errors.size() + " previous error";
      if (errors.size() > 1) {
        cause += 's';
      }
      reporter.printFatalErrorAndExit("Aborting due to " + cause + '.');
    }
  }

  /**
   * If any errors were collected, print them, then return them.
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

  private String[] jsonStringToArgs(String jsonString) {
    ArrayList<String> argsList = new ArrayList<>();
    JsonObject jsonObject = new JsonObject();

    // Parse JSON string and get top-level JSON object.
    try {
      jsonObject = JsonParser.parseString(jsonString).getAsJsonObject();
    } catch (JsonParseException e) {
      reporter.printFatalErrorAndExit(String.format("Invalid JSON string:%n %s", jsonString));
    }
    // Append input paths.
    JsonElement src = jsonObject.get("src");
    if (src == null) {
      reporter.printFatalErrorAndExit("JSON Parse Exception: field \"src\" not found.");
    }
    assert src != null;
    argsList.add(src.getAsString());
    // Append output path if given.
    JsonElement out = jsonObject.get("out");
    if (out != null) {
      argsList.add("--output-path");
      argsList.add(out.getAsString());
    }

    // If there are no other properties, return args array.
    JsonElement properties = jsonObject.get("properties");
    if (properties != null) {
      // Get the remaining properties.
      Set<Entry<String, JsonElement>> entrySet = properties.getAsJsonObject().entrySet();
      // Append the remaining properties to the args array.
      for (Entry<String, JsonElement> entry : entrySet) {
        String property = entry.getKey();
        String value = entry.getValue().getAsString();

        // Append option.
        argsList.add("--" + property);
        // Append argument for non-boolean options.
        if (!value.equals("true") || property.equals("threading")) {
          argsList.add(value);
        }
      }
    }

    return argsList.toArray(new String[0]);
  }
}
