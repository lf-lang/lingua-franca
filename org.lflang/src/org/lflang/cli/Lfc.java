/**
 * Stand-alone version of the Lingua Franca compiler (lfc).
 */

package org.lflang.cli;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.stream.Collectors;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
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
 */
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
     * <p>
     * Stores an Apache Commons CLI Option for each entry, sets it to be
     * if required if so specified, and stores whether or not to pass the
     * option to the code generator.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     */
    enum CLIOption {

        BUILD_TYPE(BuildParm.BUILD_TYPE, null, true, false,  true),
        CLEAN(BuildParm.CLEAN, "c", false, false, true),
        TARGET_COMPILER(BuildParm.TARGET_COMPILER, null, true, false, true),
        EXTERNAL_RUNTIME_PATH(BuildParm.EXTERNAL_RUNTIME_PATH, null, true, false, true),
        FEDERATED(BuildParm.FEDERATED, "f", false, false, false),
        LOGGING(BuildParm.LOGGING, null, true, false, true),
        LINT(BuildParm.LINT, "l",false, false,  true),
        NO_COMPILE(BuildParm.NO_COMPILE, "n", false, false, true),
        OUTPUT_PATH(BuildParm.OUTPUT_PATH, "o", true, false, false),
        QUIET(BuildParm.QUIET, "q", false, false,  true),
        RTI(BuildParm.RTI, "r", true, false, true),
        RUNTIME_VERSION(BuildParm.RUNTIME_VERSION, null, true, false, true),
        SCHEDULER(BuildParm.SCHEDULER, "s", true, false, true),
        THREADING(BuildParm.THREADING, "t", true, false, true),
        WORKERS(BuildParm.WORKERS, "w", true, false, true);

        /**
         * The corresponding Apache CLI Option object.
         */
        public final Option option;

        /**
         * Whether to pass this option to the code generator.
         */
        public final boolean passOn;

        /**
         * Construct a new CLIOption.
         *
         * @param parameter   The build parameter that this CLI parameter corresponds to.
         * @param shorthand   The single-character switch to use for this option. E.g.:
         *                    "-c" for "--clean".
         * @param hasArg      Whether this option has an argument. E.g.:
         *                    "--foo bar" where "bar" is the argument value.
         * @param isReq       Whether this option is required. If it is
         *                    required but not specified a menu is shown.
         * @param passOn      Whether to pass this option as a property
         *                    to the code generator.
         */
        CLIOption(BuildParm parameter, String shorthand, boolean hasArg, boolean isReq, boolean passOn) {
            this.option = new Option(shorthand, parameter.getKey(), hasArg, parameter.description);
            option.setRequired(isReq);
            this.passOn = passOn;
        }

        /**
         * Create an Apache Commons CLI Options object and add all the options.
         *
         * @return Options object that includes all the options in this enum.
         */
        public static Options getOptions() {
            Options opts = new Options();
            Arrays.asList(CLIOption.values()).forEach(o -> opts.addOption(o.option));
            return opts;
        }

        /**
         * Return a list of options that are to be passed on to the code
         * generator.
         *
         * @return List of options that must be passed on to the code gen stage.
         */
        public static List<Option> getPassedOptions() {
            return Arrays.stream(CLIOption.values())
                .filter(opt -> opt.passOn).map(opt -> opt.option)
                .collect(Collectors.toList());
        }
    }

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

    @Override
    protected Options getOptions() {
        return CLIOption.getOptions();
    }

    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    @Override
    protected void runTool(CommandLine cmd, List<Path> files) {
        Properties properties = this.filterProps(cmd, CLIOption.getPassedOptions());
        String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
        Path root = null;
        if (cmd.hasOption(pathOption)) {
            root = io.getWd().resolve(cmd.getOptionValue(pathOption)).normalize();
            if (!Files.exists(root)) { // FIXME: Create it instead?
                reporter.printFatalErrorAndExit("Output location '" + root + "' does not exist.");
            }
            if (!Files.isDirectory(root)) {
                reporter.printFatalErrorAndExit(
                    "Output location '" + root + "' is not a directory.");
            }
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(path + ": No such file or directory");
            }
        }
        for (Path path : files) {
            path = toAbsolutePath(path);
            String outputPath = getActualOutputPath(root, path).toString();
            this.fileAccess.setOutputPath(outputPath);

            final Resource resource = getResource(path);
            if (resource == null) {
                reporter.printFatalErrorAndExit(
                    path + " is not an LF file. Use the .lf file extension to denote LF files.");
            }
            else if (cmd.hasOption(CLIOption.FEDERATED.option.getOpt())) {
                if (!ASTUtils.makeFederated(resource)) {
                    reporter.printError("Unable to change main reactor to federated reactor.");
                }
            }
            validateResource(resource);

            exitIfCollectedErrors();

            LFGeneratorContext context = new MainContext(
                LFGeneratorContext.Mode.STANDALONE, CancelIndicator.NullImpl, (m, p) -> {}, properties, false,
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
