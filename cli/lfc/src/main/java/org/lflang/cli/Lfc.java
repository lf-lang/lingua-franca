package org.lflang.cli;

import com.google.inject.Inject;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Properties;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.lflang.FileConfig;
import org.lflang.TargetProperty.UnionType;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.MainContext;
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

  @Option(names = "--target-compiler", description = "Target compiler to invoke.")
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

  @Option(names = "--logging", description = "The logging level to use by the generated binary")
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
  private boolean noCompile;

  @Option(
      names = {"--verify"},
      arity = "0",
      description = "Run the generated verification models.")
  private boolean verify;

  @Option(
      names = {"--print-statistics"},
      arity = "0",
      description = "Instruct the runtime to collect and print statistics.")
  private boolean printStatistics;

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
      names = {"-t", "--threading"},
      paramLabel = "<true/false>",
      description = "Specify whether the runtime should use multi-threading" + " (true/false).")
  private String threading;

  @Option(
      names = {"-w", "--workers"},
      description = "Specify the default number of worker threads.")
  private Integer workers;

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
    // Hard code the props based on the options we want.
    Properties properties = this.getGeneratorArgs();

    try {
      // Invoke the generator on all input file paths.
      invokeGenerator(paths, outputRoot, properties);
    } catch (RuntimeException e) {
      reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
    }
  }

  /** Invoke the code generator on the given validated file paths. */
  private void invokeGenerator(List<Path> files, Path root, Properties properties) {
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
              properties,
              resource,
              this.fileAccess,
              fileConfig -> messageReporter);

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

  private Path getActualOutputPath(Path root, Path path) {
    if (root != null) {
      return root.resolve("src-gen");
    } else {
      Path pkgRoot = FileConfig.findPackageRoot(path, reporter::printWarning);
      return pkgRoot.resolve("src-gen");
    }
  }

  /**
   * Filter the command-line arguments needed by the code generator, and return them as properties.
   *
   * @return Properties for the code generator.
   */
  public Properties getGeneratorArgs() {
    Properties props = new Properties();

    if (buildType != null) {
      // Validate build type.
      if (UnionType.BUILD_TYPE_UNION.forName(buildType) == null) {
        reporter.printFatalErrorAndExit(buildType + ": Invalid build type.");
      }
      props.setProperty(BuildParm.BUILD_TYPE.getKey(), buildType);
    }

    if (clean) {
      props.setProperty(BuildParm.CLEAN.getKey(), "true");
    }

    if (externalRuntimePath != null) {
      props.setProperty(BuildParm.EXTERNAL_RUNTIME_PATH.getKey(), externalRuntimePath.toString());
    }

    if (lint) {
      props.setProperty(BuildParm.LINT.getKey(), "true");
    }

    if (logging != null) {
      // Validate log level.
      if (UnionType.LOGGING_UNION.forName(logging) == null) {
        reporter.printFatalErrorAndExit(logging + ": Invalid log level.");
      }
      props.setProperty(BuildParm.LOGGING.getKey(), logging);
    }

    if (printStatistics) {
      props.setProperty(BuildParm.PRINT_STATISTICS.getKey(), "true");
    }

    if (noCompile) {
      props.setProperty(BuildParm.NO_COMPILE.getKey(), "true");
    }

    if (verify) {
      props.setProperty(BuildParm.VERIFY.getKey(), "true");
    }

    if (targetCompiler != null) {
      props.setProperty(BuildParm.TARGET_COMPILER.getKey(), targetCompiler);
    }

    if (quiet) {
      props.setProperty(BuildParm.QUIET.getKey(), "true");
    }

    if (rti != null) {
      // Validate RTI path.
      if (!Files.exists(io.getWd().resolve(rti))) {
        reporter.printFatalErrorAndExit(rti + ": Invalid RTI path.");
      }
      props.setProperty(BuildParm.RTI.getKey(), rti.toString());
    }

    if (runtimeVersion != null) {
      props.setProperty(BuildParm.RUNTIME_VERSION.getKey(), runtimeVersion);
    }

    if (scheduler != null) {
      // Validate scheduler.
      if (UnionType.SCHEDULER_UNION.forName(scheduler) == null) {
        reporter.printFatalErrorAndExit(scheduler + ": Invalid scheduler.");
      }
      props.setProperty(BuildParm.SCHEDULER.getKey(), scheduler);
    }

    if (threading != null) {
      props.setProperty(BuildParm.THREADING.getKey(), threading);
    }

    if (workers != null) {
      props.setProperty(BuildParm.WORKERS.getKey(), workers.toString());
    }

    return props;
  }
}
