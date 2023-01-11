package org.lflang.cli;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.stream.Collectors;

import picocli.CommandLine;
import picocli.CommandLine.Command;
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
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 * @author {Atharva Patil <atharva.patil@berkeley.edu>}
 */
@Command(
    name = "lfc",
    // Automatically add usageHelp and versionHelp options.
    mixinStandardHelpOptions = true,
    // TODO: Import version from StringsBundle.properties. 
    version = "lfc 0.3.1-SNAPSHOT")
public class Lfc extends CliBaseNew {
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
     * @author Marten Lohstroh <marten@berkeley.edu>
     * @author Atharva Patil <atharva.patil@berkeley.edu>
     */

    @Option(
        names = "--build-type",
        description = "The build type to use.")
    private String buildType;

    @Option(
        names = {"-c", "--clean"},
        description = "Clean before building.")
    // boolean?
    private String clean;

    @Option(
        names = "--target-compiler",
        description = "Target compiler to invoke.")
    private String targetCompiler;

    @Option(
        names = "--external-runtime-path",
        description = "Specify an external runtime library to be used by the"
                    + " compiled binary.")
    private String externalRuntimePath;

    @Option(
        names = {"-f", "--federated"},
        description = "Treat main reactor as federated.")
    private boolean federated;

    @Option(
        names = "--logging",
        description = "The logging level to use by the generated binary")
    private String logging;

    @Option(
        names = {"-l", "--lint"},
        description = "Enable or disable linting of generated code.")
    // boolean?
    private String lint;

    @Option(
        names = {"-n", "--no-compile"},
        description = "Do not invoke target compiler.")
    // boolean?
    private boolean noCompile;

    @Option(
        names = {"-o", "--output-path"},
        defaultValue = "",
        fallbackValue = "",
        description = "Specify the root output directory.")
    private String outputPath;

    @Option(
        names = {"-q", "--quiet"},
        description = "Suppress output of the target compiler and other commands")
    // boolean?
    private String quiet;

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

    // TODO: could be boolean?
    @Option(
        names = {"-t", "--threading"},
        description = "Specify whether the runtime should use multi-threading (true/false).")
    private String threading;

    // TODO: could be int?
    @Option(
        names = {"-w", "--workers"},
        description = "Specify the default number of worker threads.")
    private String workers;

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

    /*
     * The first method in LFC that is invoked when the parent CliBase Runnable
     * class is instantiated, i.e. the first method to run after the arguments
     * are parsed.
     */
    @Override
    public void run() {
        try {
            List<Path> paths = files.stream().map(
                    io.getWd()::resolve).collect(Collectors.toList());
            runTool(paths);
        } catch (RuntimeException e) {
            reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
        }
    }

    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    @Override
    protected void runTool(List<Path> files) {
        // Hard code the props based on the options we want.
        Properties properties = this.filterPassOnProps();

        Path root = null;
        if (!outputPath.isEmpty()) {
            root = io.getWd().resolve(outputPath).normalize();
            if (!Files.exists(root)) { // FIXME: Create it instead?
                reporter.printFatalErrorAndExit(
                    "Output location '" + root + "' does not exist.");
            }
            if (!Files.isDirectory(root)) {
                reporter.printFatalErrorAndExit(
                    "Output location '" + root + "' is not a directory.");
            }
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(
                    path + ": No such file or directory");
            }
        }

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
                LFGeneratorContext.Mode.STANDALONE,
                CancelIndicator.NullImpl, (m, p) -> {}, properties, false,
                fileConfig -> errorReporter
            );

            try {
                this.generator.generate(resource, this.fileAccess, context);
            } catch (Exception e) {
                reporter.printFatalErrorAndExit("Error running generator", e);
            }

            exitIfCollectedErrors();
            // print all other issues (not errors)
            issueCollector.getAllIssues().forEach(reporter::printIssue);

            this.io.getOut().println("Code generation finished.");
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
}
