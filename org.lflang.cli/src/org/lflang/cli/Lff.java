/**
 * Stand-alone version of the Lingua Franca formatter (lff).
 */

package org.lflang.cli;

import java.io.IOException;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
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
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.ast.ToLf;
import org.lflang.util.FileUtil;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

/**
 * Standalone version of the Lingua Franca formatter (lff).
 * Based on lfc.
 *
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 * @author {Billy Bao <billybao@berkeley.edu>}
 */
public class Lff {

    /// current lff version as printed by --version
    private static final String VERSION = "0.2.2-SNAPSHOT";

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
     * required if so specified, and stores whether to pass the
     * option to the code generator.
     *
     * @author Marten Lohstroh <marten@berkeley.edu>
     * @author {Billy Bao <billybao@berkeley.edu>}
     */
    enum CLIOption {
        HELP("h", "help", false, false, "Display this information."),
        LINE_WRAP("w", "wrap", true, false, "Causes the formatter to line wrap the files to a specified length."),
        NO_RECURSE(null, "no-recurse", false, false, "Do not format files in subdirectories of the specified paths."),
        OUTPUT_PATH("o", "output-path", true, false, "If specified, outputs all formatted files into this directory instead "
            + "of overwriting the original files. Subdirectory structure will be preserved."),
        VERBOSE("v", "verbose", false, false, "Print more details on files affected."),
        VERSION(null, "version", false, false, "Print version information.");

        /**
         * The corresponding Apache CLI Option object.
         */
        public final Option option;

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
         */
        CLIOption(String opt, String longOpt, boolean hasArg, boolean isReq, String description) {
            this.option = new Option(opt, longOpt, hasArg, description);
            option.setRequired(isReq);
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
    }

    /**
     * Main function of the stand-alone compiler.
     *
     * @param args CLI arguments
     */
    public static void main(final String[] args) {
        final ReportingBackend reporter = new ReportingBackend(new Io(), "lff: ");

        // Injector used to obtain Main instance.
        final Injector injector = new LFStandaloneSetup(new LFRuntimeModule(), new LFStandaloneModule(reporter))
            .createInjectorAndDoEMFRegistration();
        // Main instance.
        final Lff main = injector.getInstance(Lff.class);
        // Apache Commons Options object configured according to available CLI arguments.
        Options options = CLIOption.getOptions();
        // CLI arguments parser.
        CommandLineParser parser = new DefaultParser();
        // Helper object for printing "help" menu.
        HelpFormatter formatter = new HelpFormatter();

        try {
            main.cmd = parser.parse(options, args, true);

            // If requested, print help and abort
            if (main.cmd.hasOption(CLIOption.HELP.option.getOpt())) {
                formatter.printHelp("lff", options);
                System.exit(0);
            }

            // If requested, print version and abort
            if (main.cmd.hasOption(CLIOption.VERSION.option.getLongOpt())) {
                System.out.println("lff " + VERSION);
                System.exit(0);
            }

            List<String> files = main.cmd.getArgList();

            if (files.size() < 1) {
                reporter.printFatalErrorAndExit("No input files.");
            }
            try {
                List<Path> paths = files.stream().map(Paths::get).collect(Collectors.toList());
                main.runFormatter(paths);
            } catch (RuntimeException e) {
                reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
            }
        } catch (ParseException e) {
            reporter.printFatalError("Unable to parse commandline arguments. Reason: " + e.getMessage());
            formatter.printHelp("lff", options);
            System.exit(1);
        }
    }

    /**
     * Check all given input paths and the output path, then invokes the formatter on all files given.
     */
    private void runFormatter(List<Path> files) {
        String pathOption = CLIOption.OUTPUT_PATH.option.getOpt();
        Path outputRoot = null;
        if (cmd.hasOption(pathOption)) {
            outputRoot = Paths.get(cmd.getOptionValue(pathOption)).toAbsolutePath().normalize();
            if (!Files.exists(outputRoot)) {
                reporter.printFatalErrorAndExit("Output location '" + outputRoot + "' does not exist.");
            }
            if (!Files.isDirectory(outputRoot)) {
                reporter.printFatalErrorAndExit("Output location '" + outputRoot + "' is not a directory.");
            }
        }

        for (Path path : files) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(path + ": No such file or directory");
            }
        }

        if (cmd.hasOption(CLIOption.LINE_WRAP.option.getOpt())) {
            ToLf.instance.setLineWrap(Integer.parseInt(cmd.getOptionValue(CLIOption.LINE_WRAP.option.getOpt())));
        }

        for (Path path : files) {
            if (cmd.hasOption(CLIOption.VERBOSE.option.getOpt())) {
                reporter.printInfo("Formatting " + path + ":");
            }
            path = path.toAbsolutePath();
            if (Files.isDirectory(path) && !cmd.hasOption(CLIOption.NO_RECURSE.option.getLongOpt())) {
                formatRecursive(Paths.get("."), path, outputRoot);
            } else {
                if (outputRoot == null) {
                    formatSingleFile(path, path);
                } else {
                    formatSingleFile(path, outputRoot.resolve(path.getFileName()));
                }
            }
        }
        reporter.printInfo("Done formatting.");
    }

    /**
     * Invoke the formatter on all files in a directory recursively.
     * @param curPath Current relative path from inputRoot.
     * @param inputRoot Root directory of input files.
     * @param outputRoot Root output directory.
     */
    private void formatRecursive(Path curPath, Path inputRoot, Path outputRoot) {
        Path curDir = inputRoot.resolve(curPath);
        try (var dirStream = Files.newDirectoryStream(curDir)) {
            for (Path path : dirStream) {
                Path newPath = curPath.resolve(path.getFileName());
                if (Files.isDirectory(path)) {
                    formatRecursive(newPath, inputRoot, outputRoot);
                } else {
                    if (outputRoot == null) {
                        formatSingleFile(path, path);
                    } else {
                        formatSingleFile(path, outputRoot.resolve(newPath));
                    }
                }
            }
        } catch (IOException e) {
            reporter.printError("Error reading directory " + curDir + ": " + e.getMessage());
        }
    }

    /**
     * Load and validate a single file, then formats it and outputs to the given outputPath.
     */
    private void formatSingleFile(Path file, Path outputPath) {
        file = file.normalize();
        outputPath = outputPath.normalize();
        final Resource resource = getValidatedResource(file);
        if (resource == null) return; // not an LF file, nothing to do here

        exitIfCollectedErrors();

        String res = ToLf.instance.doSwitch(resource.getContents().get(0));
        try {
            FileUtil.writeToFile(res, outputPath, true);
        } catch (IOException e) {
            if (e instanceof FileAlreadyExistsException) {
                // only happens if a subdirectory is named with ".lf" at the end
                reporter.printFatalErrorAndExit("Error writing to " + outputPath + ": file already exists. Make sure that no file or directory "
                                                    + "within provided input paths have the same relative paths.");
            }
            reporter.printFatalErrorAndExit("Error writing to " + outputPath + ": " + e.getMessage());
        }

        exitIfCollectedErrors();
        issueCollector.getAllIssues().forEach(reporter::printIssue);
        if (cmd.hasOption(CLIOption.VERBOSE.option.getOpt())) {
            String msg = "Formatted " + file;
            if (file != outputPath) msg += " -> " + outputPath;
            reporter.printInfo(msg);
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
     * these are recorded using the issue collector. Returns null if path is not an LF file.
     *
     * @param path Path to the resource to validate.
     * @return A validated resource
     */
    // visible in tests
    public Resource getValidatedResource(Path path) {
        final Resource resource = getResource(path);
        if (resource == null) return null;

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
            if (cmd.hasOption(CLIOption.VERBOSE.option.getOpt())) {
                reporter.printInfo("Skipped " + path + ": not an LF file");
            }
            return null;
        }
    }
}
