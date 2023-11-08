package org.lflang.cli;

import com.google.inject.Inject;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.FileConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.Argument;
import org.lflang.generator.GeneratorArguments;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.MainContext;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.TracingProperty.TracingOptions;
import org.lflang.target.property.VerifyProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.BuildTypeType;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.target.property.type.LoggingType;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.SchedulerType;
import org.lflang.target.property.type.SchedulerType.Scheduler;
import picocli.CommandLine.ArgGroup;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;

/**
 * Standalone version of the Lingua Franca compiler (lfc).
 *
 * @author Marten Lohstroh
 * @author Christian Menard
 * @author Atharva Patil
 */
@Command(
    name = "lfc",
    // Enable usageHelp (--help) and versionHelp (--version) options.
    mixinStandardHelpOptions = true,
    versionProvider = VersionProvider.class)
public class Lfc extends CliBase {
  /** Injected code generator. */
  @Inject private GeneratorDelegate generator;

  /** Injected file access object. */
  @Inject private JavaIoFileSystemAccess fileAccess;

  /*
   * Supported CLI options.
   */

  @Option(names = "--build-type", description = "The build type to use.")
  private String buildType;

  @Option(
      names = {"-c", "--clean"},
      arity = "0",
      description = "Clean before building.")
  private boolean clean;

  @Option(names = "--compiler", description = "Target compiler to invoke.")
  private String targetCompiler;

  @Option(
      names = "--external-runtime-path",
      description = "Specify an external runtime library to be used by the" + " compiled binary.")
  private Path externalRuntimePath;

  @Option(
      names = {"-f", "--federated"},
      arity = "0",
      description = "Treat main reactor as federated.")
  private boolean federated;

  @Option(
      names = {"--hierarchical-bin"},
      arity = "0",
      description =
          "Organize the generated binaries hierarchically, reflecting the structure of the source"
              + " tree.")
  private boolean hierarchicalBin;

  @Option(names = "--logging", description = "The logging level to use by the generated binary.")
  private String logging;

  @Option(
      names = {"-l", "--lint"},
      arity = "0",
      description = "Enable linting of generated code.")
  private boolean lint;

  @Option(
      names = {"-n", "--no-compile"},
      arity = "0",
      description = "Do not invoke target compiler.")
  private Boolean noCompile;

  @Option(
      names = {"--verify"},
      arity = "0",
      description = "Run the generated verification models.")
  private Boolean verify;

  @Option(
      names = {"--print-statistics"},
      arity = "0",
      description = "Instruct the runtime to collect and print statistics.")
  private Boolean printStatistics;

  @Option(
      names = {"-q", "--quiet"},
      arity = "0",
      description = "Suppress output of the target compiler and other commands")
  private boolean quiet;

  @Option(
      names = {"-r", "--rti"},
      description = "Specify the location of the RTI.")
  private Path rti;

  @Option(
      names = "--runtime-version",
      description =
          "Specify the version of the runtime library used for" + " compiling LF programs.")
  private String runtimeVersion;

  @Option(
      names = {"-s", "--scheduler"},
      description = "Specify the runtime scheduler (if supported).")
  private String scheduler;

  @Option(
      names = {"--tracing"},
      arity = "0",
      description = "Specify whether to enable run-time tracing (if supported).")
  private Boolean tracing;

  /** Mutually exclusive options related to threading. */
  static class ThreadingMutuallyExclusive {
    @Option(
        names = "--single-threaded",
        arity = "0",
        description = "Specify whether the runtime should be single-threaded.")
    private boolean singleThreaded;

    @Option(
        names = {"-w", "--workers"},
        description = "Specify the number of worker threads.")
    private Integer workers;
  }

  @ArgGroup(exclusive = true, multiplicity = "0..1")
  ThreadingMutuallyExclusive threading;

  /**
   * Main function of the stand-alone compiler. Caution: this will invoke System.exit.
   *
   * @param args CLI arguments
   */
  public static void main(final String[] args) {
    main(Io.SYSTEM, args);
  }

  /**
   * Main function of the standalone compiler, with a custom IO.
   *
   * @param io IO streams.
   * @param args Command-line arguments.
   */
  public static void main(Io io, final String... args) {
    cliMain("lfc", Lfc.class, io, args);
  }

  /** Load the resource, validate it, and, invoke the code generator. */
  @Override
  public void doRun() {
    List<Path> paths = getInputPaths();
    final Path outputRoot = getOutputRoot();
    var args = this.getArgs();

    try {
      // Invoke the generator on all input file paths.
      invokeGenerator(paths, outputRoot, args);
    } catch (RuntimeException e) {
      reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
    }
  }

  /** Invoke the code generator on the given validated file paths. */
  private void invokeGenerator(List<Path> files, Path root, GeneratorArguments args) {
    for (Path path : files) {
      path = toAbsolutePath(path);

      String outputPath = getActualOutputPath(root, path).toString();
      this.fileAccess.setOutputPath(outputPath);

      final Resource resource = getResource(path);
      if (resource == null) {
        reporter.printFatalErrorAndExit(
            path + " is not an LF file. Use the .lf file extension to" + " denote LF files.");
      } else if (federated) {
        if (!ASTUtils.makeFederated(resource)) {
          reporter.printError("Unable to change main reactor to federated reactor.");
        }
      }

      validateResource(resource);
      exitIfCollectedErrors();

      LFGeneratorContext context =
          new MainContext(
              LFGeneratorContext.Mode.STANDALONE,
              CancelIndicator.NullImpl,
              (m, p) -> {},
              args,
              resource,
              this.fileAccess,
              fileConfig -> messageReporter);

      // Exit if there were problems creating the main context.
      exitIfCollectedErrors();

      try {
        this.generator.generate(resource, this.fileAccess, context);
      } catch (Exception e) {
        reporter.printFatalErrorAndExit("Error running generator", e);
      }

      exitIfCollectedErrors();
      // Print all other issues (not errors).
      issueCollector.getAllIssues().forEach(reporter::printIssue);

      messageReporter.nowhere().info("Code generation finished.");
    }
  }

  /** Return a resolved path that designates where to write files to. */
  private Path getActualOutputPath(Path root, Path path) {
    if (root != null) {
      return root.resolve("src-gen");
    } else {
      Path pkgRoot = FileConfig.findPackageRoot(path, reporter::printWarning);
      return pkgRoot.resolve("src-gen");
    }
  }

  /**
   * Return a build type if one has been specified via the CLI arguments, or {@code null} otherwise.
   */
  private BuildType getBuildType() {
    BuildType resolved = null;
    if (buildType != null) {
      // Validate build type.
      resolved = new BuildTypeType().forName(buildType);
      if (resolved == null) {
        reporter.printFatalErrorAndExit(buildType + ": Invalid build type.");
      }
    }
    return resolved;
  }

  /**
   * Return a log level if one has been specified via the CLI arguments, or {@code null} otherwise.
   */
  private LogLevel getLogging() {
    LogLevel resolved = null;
    if (logging != null) {
      // Validate log level.
      resolved = new LoggingType().forName(logging);
      if (resolved == null) {
        reporter.printFatalErrorAndExit(logging + ": Invalid log level.");
      }
    }
    return resolved;
  }

  /**
   * Return a URI that points to the RTI if one has been specified via the CLI arguments, or {@code
   * null} otherwise.
   */
  private URI getRtiUri() {
    URI uri = null;
    if (rti != null) {
      // Validate RTI path.
      if (!Files.exists(io.getWd().resolve(rti))) {
        reporter.printFatalErrorAndExit(rti + ": Invalid RTI path.");
      }
      uri = rti.toUri();
    }
    return uri;
  }

  /** Return a scheduler one has been specified via the CLI arguments, or {@code null} otherwise. */
  private Scheduler getScheduler() {
    Scheduler resolved = null;
    if (scheduler != null) {
      // Validate scheduler.
      resolved = new SchedulerType().forName(scheduler);
      if (resolved == null) {
        reporter.printFatalErrorAndExit(scheduler + ": Invalid scheduler.");
      }
    }
    return resolved;
  }

  /**
   * Return a URI that points to an external runtime if one has been specified via the CLI
   * arguments, or {@code null} otherwise.
   */
  private URI getExternalRuntimeUri() {
    URI externalRuntimeUri = null;
    if (externalRuntimePath != null) {
      externalRuntimeUri = externalRuntimePath.toUri();
    }
    return externalRuntimeUri;
  }

  /**
   * Return tracing options if tracing has been explicitly disabled or enabled via the CLI
   * arguments, or {@code null} otherwise.
   */
  private TracingOptions getTracingOptions() {
    if (tracing != null) {
      return new TracingOptions(tracing);
    } else {
      return null;
    }
  }

  /** Return the single threaded mode has been specified, or {@code null} if none was specified. */
  private Boolean getSingleThreaded() {
    Boolean singleThreaded = null;
    // Set one of the mutually-exclusive threading options.
    if (threading != null) {
      singleThreaded = threading.singleThreaded;
    }
    return singleThreaded;
  }

  /** Return the number of workers specified, or {@code null} if none was specified. */
  private Integer getWorkers() {
    Integer workers = null;
    // Set one of the mutually-exclusive threading options.
    if (threading != null) {
      workers = threading.workers;
    }
    return workers;
  }
  /** Check the values of the commandline arguments and return them. */
  public GeneratorArguments getArgs() {

    return new GeneratorArguments(
        clean,
        getExternalRuntimeUri(),
        hierarchicalBin,
        getJsonObject(),
        lint,
        quiet,
        getRtiUri(),
        List.of(
            new Argument<>(BuildTypeProperty.INSTANCE, getBuildType()),
            new Argument<>(CompilerProperty.INSTANCE, targetCompiler),
            new Argument<>(LoggingProperty.INSTANCE, getLogging()),
            new Argument<>(PrintStatisticsProperty.INSTANCE, printStatistics),
            new Argument<>(NoCompileProperty.INSTANCE, noCompile),
            new Argument<>(VerifyProperty.INSTANCE, verify),
            new Argument<>(RuntimeVersionProperty.INSTANCE, runtimeVersion),
            new Argument<>(SchedulerProperty.INSTANCE, getScheduler()),
            new Argument<>(SingleThreadedProperty.INSTANCE, getSingleThreaded()),
            new Argument<>(TracingProperty.INSTANCE, getTracingOptions()),
            new Argument<>(WorkersProperty.INSTANCE, getWorkers())));
  }
}
