package org.lflang.cli;

import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Path;
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
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.ErrorReporter;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.LocalStrings;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.util.FileUtil;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

/**
 * Base class for standalone CLI applications.
 *
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 * @author {Billy Bao <billybao@berkeley.edu>}
 */
public abstract class CliBase {

    /**
     * Used to collect all errors that happen during validation/generation.
     */
    @Inject
    protected IssueCollector issueCollector;
    /**
     * Used to report error messages at the end.
     */
    @Inject
    protected ReportingBackend reporter;
    /**
     * Used to report error messages at the end.
     */
    @Inject
    protected ErrorReporter errorReporter;
    /**
     * IO context of this run.
     */
    @Inject
    protected Io io;
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

    /** Name of the program, eg "lfc". */
    private final String toolName;

    protected CliBase(String toolName) {
        this.toolName = toolName;
    }

    protected static void cliMain(String toolName, Class<? extends CliBase> toolClass, Io io, String[] args) {
        // Injector used to obtain Main instance.
        final Injector injector = getInjector(toolName, io);
        // Main instance.
        final CliBase main = injector.getInstance(toolClass);
        main.runMain(args);
    }

    protected static Injector getInjector(String toolName, Io io) {
        final ReportingBackend reporter = new ReportingBackend(io, toolName + ": ");

        // Injector used to obtain Main instance.
        return new LFStandaloneSetup(new LFRuntimeModule(), new LFStandaloneModule(reporter, io))
            .createInjectorAndDoEMFRegistration();
    }


    /**
     * Main function of the tool.
     *
     * @param args Command-line arguments.
     */
    protected void runMain(final String... args) {

        // Main instance.
        // Apache Commons Options object configured to according to available CLI arguments.
        Options options = getOptions();
        // CLI arguments parser.
        CommandLineParser parser = new DefaultParser();
        // Helper object for printing "help" menu.
        HelpFormatter formatter = new HelpFormatter();

        final Option helpOption = new Option("h", "help", false, BuildParm.HELP.description);
        final Option versionOption = new Option("version", "version", false, BuildParm.VERSION.description);

        options.addOption(helpOption);
        options.addOption(versionOption);

        CommandLine cmd;
        try {
            cmd = parser.parse(options, args, false);
        } catch (ParseException e) {
            reporter.printFatalError(
                "Unable to parse command-line arguments. Reason: " + e.getMessage() + "\n"
                    + "The full command-line was: " + Arrays.toString(args));
            printHelp(options, formatter, io.getErr());
            io.callSystemExit(1);
            return;
        }

        // If requested, print help and abort
        if (cmd.hasOption(helpOption.getOpt())) {
            printHelp(options, formatter, io.getOut());
            io.callSystemExit(0);
        }

        // If requested, print version and abort
        if (cmd.hasOption(versionOption.getLongOpt())) {
            io.getOut().println(toolName + " " + LocalStrings.VERSION);
            io.callSystemExit(0);
        }

        List<String> files = cmd.getArgList();

        if (files.size() < 1) {
            reporter.printFatalErrorAndExit("No input files.");
        }
        try {
            List<Path> paths = files.stream().map(io.getWd()::resolve).collect(Collectors.toList());
            runTool(cmd, paths);
        } catch (RuntimeException e) {
            reporter.printFatalErrorAndExit("An unexpected error occurred:", e);
        }
        io.callSystemExit(0);
    }

    protected abstract Options getOptions();
    protected abstract void runTool(CommandLine cmd, List<Path> inputFiles);

    // Print help on the correct output stream. Unfortunately the library doesn't have
    // a more convenient overload.
    private void printHelp(Options options, HelpFormatter formatter, PrintStream out) {
        try (PrintWriter pw = new PrintWriter(out)) {
            formatter.printHelp(pw,
                formatter.getWidth(),
                toolName,
                null,
                options,
                formatter.getLeftPadding(),
                formatter.getDescPadding(),
                null,
                false);
        }
    }

    /** Resolve to an absolute path, in the given {@link #io} context. */
    protected Path toAbsolutePath(Path other) {
        return io.getWd().resolve(other).toAbsolutePath();
    }

    /**
     * Store command-line arguments as properties, to be passed on to the runtime.
     *
     * @param passOptions Which options should be passed to the runtime.
     * @return Provided arguments in cmd as properties, which should be passed to the runtime.
     */
    protected Properties filterProps(CommandLine cmd, List<Option> passOptions) {
        Properties props = new Properties();
        for (Option o : cmd.getOptions()) {
            if (passOptions.contains(o)) {
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
     * If some errors were collected, print them and abort execution. Otherwise, return.
     */
    protected void exitIfCollectedErrors() {
        if (issueCollector.getErrorsOccurred()) {
            // if there are errors, don't print warnings.
            List<LfIssue> errors = printErrorsIfAny();
            String cause = errors.size() == 1 ? "previous error"
                                              : errors.size() + " previous errors";
            reporter.printFatalErrorAndExit("Aborting due to " + cause);
        }
    }

    /**
     * If any errors were collected, print them, then return them.
     * @return A list of collected errors.
     */
    public List<LfIssue> printErrorsIfAny() {
        List<LfIssue> errors = issueCollector.getErrors();
        errors.forEach(reporter::printIssue);
        return errors;
    }

    /**
     * Validates a given resource. If issues arise during validation,
     * these are recorded using the issue collector.
     *
     * @param resource The resource to validate.
     */
    public void validateResource(Resource resource) {
        assert resource != null;

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
    }

    /**
     * Obtains a resource from a path. Returns null if path is not an LF file.
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
