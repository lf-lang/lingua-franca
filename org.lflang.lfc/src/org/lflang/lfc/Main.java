/**
 * Stand-alone version of the Lingua Franca compiler (lfc).
 */

package org.lflang.lfc;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.stream.Collectors;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.MainContext;
import org.lflang.util.FileUtil;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

/**
 * Standalone version of the Lingua Franca compiler (lfc).
 *
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 */
public class Main {

    /// current lfc version as printed by --version
    private static final String VERSION = "0.2.0";

    /**
     * Object for interpreting command line arguments.
     */
    protected CommandLine cmd;

    /**
     * Injected resource provider.
     */
    @Inject
    private Provider<ResourceSet> resourceSetProvider;

    /**
     * Injected resource validator.
     */
    @Inject
    private IResourceValidator validator;

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

    /**
     * Used to collect all errors that happen during validation/generation.
     */
    @Inject
    private IssueCollector issueCollector;

    /**
     * Used to report error messages at the end.
     */
    @Inject
    private ReportingBackend reporter;


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
        CLEAN("c", "clean", false, false, "Clean before building.", true),
        COMPILER(null, "target-compiler", true, false, "Target compiler to invoke.", true),
        EXTERNAL_RUNTIME_PATH(null, "external-runtime-path", true, false, "Specify an external runtime library to be used by the compiled binary.", true),
        FEDERATED("f", "federated", false, false, "Treat main reactor as federated.", false),
        HELP("h", "help", false, false, "Display this information.", true),
        LINT("l", "lint", false, false, "Enable or disable linting of generated code.", true),
        NO_COMPILE("n", "no-compile", false, false, "Do not invoke target compiler.", true),
        OUTPUT_PATH("o", "output-path", true, false, "Specify the root output directory.", false),
        QUIET("q", "quiet", false, false, "Suppress output of the target compiler and other commands", true),
        RTI("r", "rti", true, false, "Specify the location of the RTI.", true),
        RUNTIME_VERSION(null, "runtime-version", true, false, "Specify the version of the runtime library used for compiling LF programs.", true),
        SCHEDULER("s", "scheduler", true, false, "Specify the runtime scheduler (if supported).", true),
        THREADING("t", "threading", true, false, "Specify whether the runtime should use multi-threading (true/false).", true),
        VERSION(null, "version", false, false, "Print version information.", false),
        WORKERS("w", "workers", true, false, "Specify the default number of worker threads.", true);

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
         * @param opt         The short option name. E.g.: "f" denotes a flag
         *                    "-f".
         * @param longOpt     The long option name. E.g.: "foo" denotes a flag
         *                    "--foo".
         * @param hasArg      Whether or not this option has an argument. E.g.:
         *                    "--foo bar" where "bar" is the argument value.
         * @param isReq       Whether or not this option is required. If it is
         *                    required but not specified a menu is shown.
         * @param description The description used in the menu.
         * @param passOn      Whether or not to pass this option as a property
         *                    to the code generator.
         */
        CLIOption(String opt, String longOpt, boolean hasArg, boolean isReq, String description, boolean passOn) {
            this.option = new Option(opt, longOpt, hasArg, description);
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
     *
     * @param args CLI arguments
     */
    public static void main(final String[] args) {
        final ReportingBackend reporter = new ReportingBackend(new Io());

        // Injector used to obtain Main instance.
        final Injector injector = new LFStandaloneSetup(new LFRuntimeModule(), new LFStandaloneModule(reporter))
            .createInjectorAndDoEMFRegistration();
        // Main instance.
        final Main main = injector.getInstance(Main.class);
        // Apache Commons Options object configured to according to available CLI arguments.
        Options options = CLIOption.getOptions();
        // CLI arguments parser.
        CommandLineParser parser = new DefaultParser();
        // Helper object for printing "help" menu.
        HelpFormatter formatter = new HelpFormatter();

        try {
            main.cmd = parser.parse(options, args, true);

            // If requested, print help and abort
            if (main.cmd.hasOption(CLIOption.HELP.option.getOpt())) {
                formatter.printHelp("lfc", options);
                System.exit(0);
            }

            // If requested, print version and abort
            if (main.cmd.hasOption(CLIOption.VERSION.option.getLongOpt())) {
                System.out.println("lfc " + VERSION);
                System.exit(0);
            }

            List<String> files = main.cmd.getArgList();

            if (files.size() < 1) {
                reporter.printFatalErrorAndExit("No input files.");
            }
            try {
                List<Path> paths = files.stream().map(Paths::get).collect(Collectors.toList());
                main.runGenerator(paths, injector);
            } catch (RuntimeException e) {
                reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
            }
        } catch (ParseException e) {
            reporter.printFatalError("Unable to parse commandline arguments. Reason: " + e.getMessage());
            formatter.printHelp("lfc", options);
            System.exit(1);
        }
    }

    /**
     * Store arguments as properties, to be passed on to the generator.
     */
    protected Properties getProps(CommandLine cmd) {
        Properties props = new Properties();
        List<Option> passOn = CLIOption.getPassedOptions();
        for (Option o : cmd.getOptions()) {
            if (passOn.contains(o)) {
                String value = "";
                if (o.hasArg()) {
                    value = o.getValue();
                }
                props.setProperty(o.getLongOpt(), value);
            }
        }
        return props;
    }

    /**
     * Load the resource, validate it, and, invoke the code generator.
     */
    private void runGenerator(List<Path> files, Injector injector) {
        Properties properties = this.getProps(cmd);
        String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
        Path root = null;
        if (cmd.hasOption(pathOption)) {
            root = Paths.get(cmd.getOptionValue(pathOption)).normalize();
            if (!Files.exists(root)) { // FIXME: Create it instead?
                reporter.printFatalErrorAndExit("Output location '" + root + "' does not exist.");
            }
            if (!Files.isDirectory(root)) {
                reporter.printFatalErrorAndExit("Output location '" + root + "' is not a directory.");
            }
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(path + ": No such file or directory");
            }
        }
        for (Path path : files) {
            path = path.toAbsolutePath();
            Path pkgRoot = FileConfig.findPackageRoot(path, reporter::printWarning);
            String resolved;
            if (root != null) {
                resolved = root.resolve("src-gen").toString();
            } else {
                resolved = pkgRoot.resolve("src-gen").toString();
            }
            this.fileAccess.setOutputPath(resolved);

            final Resource resource = getValidatedResource(path);

            exitIfCollectedErrors();

            LFGeneratorContext context = new MainContext(
                LFGeneratorContext.Mode.STANDALONE, CancelIndicator.NullImpl, (m, p) -> {}, properties, false,
                fileConfig -> injector.getInstance(ErrorReporter.class)
            );

            this.generator.generate(resource, this.fileAccess, context);

            exitIfCollectedErrors();
            // print all other issues (not errors)
            issueCollector.getAllIssues().forEach(reporter::printIssue);

            System.out.println("Code generation finished.");
        }
    }


    /**
     * If some errors were collected, print them and abort execution. Otherwise return.
     */
    private void exitIfCollectedErrors() {
        if (issueCollector.getErrorsOccurred() ) {
            // if there are errors, don't print warnings.
            List<LfIssue> errors = printErrorsIfAny();
            String cause = errors.size() == 1 ? "previous error"
                                              : errors.size() + " previous errors";
            reporter.printFatalErrorAndExit("Aborting due to " + cause);
        }
    }

    // visible in tests
    public List<LfIssue> printErrorsIfAny() {
        List<LfIssue> errors = issueCollector.getErrors();
        errors.forEach(reporter::printIssue);
        return errors;
    }

    /**
     * Given a path, obtain a resource and validate it. If issues arise during validation,
     * these are recorded using the issue collector.
     *
     * @param path Path to the resource to validate.
     * @return A validated resource
     */
    // visible in tests
    public Resource getValidatedResource(Path path) {
        final Resource resource = getResource(path);
        assert resource != null;

        if (cmd != null && cmd.hasOption(CLIOption.FEDERATED.option.getOpt())) {
            if (!ASTUtils.makeFederated(resource)) {
                reporter.printError("Unable to change main reactor to federated reactor.");
            }
        }

        List<Issue> issues = this.validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl);

        for (Issue issue : issues) {
            URI uri = issue.getUriToProblem(); // Issues may also relate to imported resources.
            try {
                issueCollector.accept(new LfIssue(issue.getMessage(), issue.getSeverity(),
                                                  issue.getLineNumber(), issue.getColumn(),
                                                  issue.getLineNumberEnd(), issue.getColumnEnd(),
                                                  issue.getLength(), FileUtil.toPath(uri)));
            } catch (IOException e) {
                reporter.printError("Unable to convert '" + uri + "' to path." + e);
            }
        }
        return resource;
    }

    private Resource getResource(Path path) {
        final ResourceSet set = this.resourceSetProvider.get();
        try {
            return set.getResource(URI.createFileURI(path.toString()), true);
        } catch (RuntimeException e) {
            reporter.printFatalErrorAndExit(path + " is not an LF file. Use the .lf file extension to denote LF files.");
            return null;
        }
    }
}
