package org.lflang.cli;


import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Model.OptionSpec;
import picocli.CommandLine.Option;
import picocli.CommandLine.Parameters;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.ASTUtils;
import org.lflang.FileConfig;

import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.MainContext;

import com.google.inject.Inject;

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
    /**
     * Injected code generator.
     */
    @Inject
    private GeneratorDelegate generator;

    /**
     * Injected file access object.
     */
    @Inject
    private JavaIoFileSystemAccess fileAccess;

    public Lfc() {
        super("lfc");
    }

    /**
     * Supported CLI options.
     *
     * @author Marten Lohstroh
     * @author Atharva Patil
     */

    @Option(
        names = "--build-type",
        description = "The build type to use.")
    private String buildType;

    @Option(
        names = {"-c", "--clean"},
        arity = "0",
        description = "Clean before building.")
    private boolean clean;

    @Option(
        names = "--target-compiler",
        description = "Target compiler to invoke.")
    private String targetCompiler;

    @Option(
        names = "--external-runtime-path",
        description = "Specify an external runtime library to be used by the"
                    + " compiled binary.")
    private Path externalRuntimePath;

    @Option(
        names = {"-f", "--federated"},
        arity = "0",
        description = "Treat main reactor as federated.")
    private boolean federated;

    @Option(
        names = "--logging",
        description = "The logging level to use by the generated binary")
    private String logging;

    @Option(
        names = {"-l", "--lint"},
        arity = "0",
        description = "Enable or disable linting of generated code.")
    private boolean lint;

    @Option(
        names = {"-n", "--no-compile"},
        arity = "0",
        description = "Do not invoke target compiler.")
    private boolean noCompile;

    @Option(
        names = {"-q", "--quiet"},
        arity = "0",
        description = 
            "Suppress output of the target compiler and other commands")
    private boolean quiet;

    @Option(
        names = {"-r", "--rti"},
        description = "Specify the location of the RTI.")
    private String rti;

    @Option(
        names = "--runtime-version",
        description = "Specify the version of the runtime library used for"
                    + " compiling LF programs.")
    private String runtimeVersion;

    @Option(
        names = {"-s", "--scheduler"},
        description = "Specify the runtime scheduler (if supported).")
    private String scheduler;

    @Option(
        names = {"-t", "--threading"},
        paramLabel = "<true/false>",
        description = "Specify whether the runtime should use multi-threading"
                    + " (true/false).")
    private String threading;

    @Option(
        names = {"-w", "--workers"},
        description = "Specify the default number of worker threads.")
    private int workers;

    /**
     * Main function of the stand-alone compiler.
     * Caution: this will invoke System.exit.
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

    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    @Override
    public void run() {
        List<Path> paths = getInputPaths();
        final Path outputRoot = getOutputRoot();
        // Hard code the props based on the options we want.
        Properties properties = this.filterPassOnProps();

        try {
            // Invoke the generator on all input file paths.
            invokeGenerator(paths, outputRoot, properties);
        } catch (RuntimeException e) {
            reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
        }
    }

    /**
     * Invoke the code generator on the given validated file paths.
     */
    private void invokeGenerator(
            List<Path> files, Path root, Properties properties) {
        for (Path path : files) {
            path = toAbsolutePath(path);
            String outputPath = getActualOutputPath(root, path).toString();
            this.fileAccess.setOutputPath(outputPath);

            final Resource resource = getResource(path);
            if (resource == null) {
                reporter.printFatalErrorAndExit(path 
                    + " is not an LF file. Use the .lf file extension to"
                    + " denote LF files.");
            } else if (federated) {
                if (!ASTUtils.makeFederated(resource)) {
                    reporter.printError(
                        "Unable to change main reactor to federated reactor.");
                }
            }

            validateResource(resource);
            exitIfCollectedErrors();

            LFGeneratorContext context = new MainContext(
                LFGeneratorContext.Mode.STANDALONE, CancelIndicator.NullImpl,
                (m, p) -> {}, properties, resource, this.fileAccess,
                fileConfig -> errorReporter
            );

            try {
                this.generator.generate(resource, this.fileAccess, context);
            } catch (Exception e) {
                reporter.printFatalErrorAndExit("Error running generator", e);
            }

            exitIfCollectedErrors();
            // Print all other issues (not errors).
            issueCollector.getAllIssues().forEach(reporter::printIssue);

            this.io.getOut().println("Code generation finished.");
        }
    }

    private Path getActualOutputPath(Path root, Path path) {
        if (root != null) {
            return root.resolve("src-gen");
        } else {
            Path pkgRoot = FileConfig.findPackageRoot(
                path, reporter::printWarning);
            return pkgRoot.resolve("src-gen");
        }
    }

    /**
     * Filter the command-line arguments needed by the code generator, and
     * return them as properties.
     *
     * @return Properties for the code generator.
     */
    protected Properties filterPassOnProps() {
        // Parameters corresponding to the options that need to be passed on to
        // the generator as properties.
        final Set<String> passOnParams = Stream.of(
            BuildParm.BUILD_TYPE,
            BuildParm.CLEAN,
            BuildParm.TARGET_COMPILER,
            BuildParm.EXTERNAL_RUNTIME_PATH,
            BuildParm.LOGGING,
            BuildParm.LINT,
            BuildParm.NO_COMPILE,
            BuildParm.QUIET,
            BuildParm.RTI,
            BuildParm.RUNTIME_VERSION,
            BuildParm.SCHEDULER,
            BuildParm.THREADING,
            BuildParm.WORKERS)
        .map(param -> param.getKey())
        .collect(Collectors.toUnmodifiableSet());

        Properties props = new Properties();

        for (OptionSpec option : spec.options()) {
            String optionName = option.longestName();
            // Check whether this option needs to be passed on to the code
            // generator as a property.
            if (passOnParams.contains(optionName)) {
                String value = "";
                // Boolean or Integer option.
                if (option.getValue() instanceof Boolean ||
                        option.getValue() instanceof Integer) {
                    value = String.valueOf(option.getValue());
                // String option.
                } else if (option.getValue() instanceof String) {
                    value = option.getValue();
                // Path option.
                } else if (option.getValue() instanceof Path) {
                    value = option.getValue().toString();
                }
                props.setProperty(optionName, value);
            }
        }
        return props;
    }
}
