package org.lflang.cli;

import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;
import java.util.stream.Collectors;

import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Model.CommandSpec;
import picocli.CommandLine.Model.OptionSpec;
import picocli.CommandLine.Option;
import picocli.CommandLine.Parameters;
import picocli.CommandLine.ParseResult;
import picocli.CommandLine.Spec;

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
 * @author {Atharva Patil <atharva.patil@berkeley.edu>}
 */
public abstract class CliBase implements Runnable {
    /**
     * Models a command specification, including the options, positional
     * parameters and subcommands supported by the command.
     */
    @Spec CommandSpec spec;

    /**
     * Options and parameters present in both Lfc and Lff.
     */
    @Parameters(
        arity = "1..",
        paramLabel = "FILES",
        description = "Paths of the files to run the formatter on.")
    protected List<Path> files;

    @Option(
        names = {"-o", "--output-path"},
        defaultValue = "",
        fallbackValue = "",
        description = "Specify the root output directory.")

    private Path outputPath;
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

    protected static void cliMain(
            String toolName, Class<? extends CliBase> toolClass,
            Io io, String[] args) {
        // Injector used to obtain Main instance.
        final Injector injector = getInjector(toolName, io);
        // Main instance.
        final CliBase main = injector.getInstance(toolClass);
        // Parse arguments and execute main logic.
        CommandLine cmd = new CommandLine(main)
            .setOut(new PrintWriter(io.getOut()))
            .setErr(new PrintWriter(io.getErr()));
        int exitCode = cmd.execute(args);
        io.callSystemExit(exitCode);
    }

    public abstract void run();

    protected static Injector getInjector(String toolName, Io io) {
        final ReportingBackend reporter 
            = new ReportingBackend(io, toolName + ": ");

        // Injector used to obtain Main instance.
        return new LFStandaloneSetup(
            new LFRuntimeModule(),
            new LFStandaloneModule(reporter, io)
        ).createInjectorAndDoEMFRegistration();
    }

    /**
     * Resolve to an absolute path, in the given {@link #io} context.
     */
    protected Path toAbsolutePath(Path other) {
        return io.getWd().resolve(other).toAbsolutePath();
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
        final List<BuildParm> passOnParams = Arrays.asList(
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
            BuildParm.WORKERS
        );

        Properties props = new Properties();
        ParseResult pr = spec.commandLine().getParseResult();

        passOnParams.forEach((param) -> {
            // Get the option with the specified name, or null if no option
            // with that name was matched on the command line.
            OptionSpec matchedOption = pr.matchedOption(param.getKey());
            if (matchedOption != null) {
                String value = "";
                // If a boolean option was set, its value is true.
                if (matchedOption.getValue() instanceof Boolean) {
                    value = "true";
                } else {
                    value = matchedOption.getValue();
                }
                props.setProperty(matchedOption.longestName(), value);
            }
        });

        return props;
    }

    /**
     * Returns the validated input paths.
     *
     * @return Validated input paths.
     */
    protected List<Path> getInputPaths() {
        List<Path> paths = files.stream()
            .map(io.getWd()::resolve)
            .collect(Collectors.toList());

        for (Path path : paths) {
            if (!Files.exists(path)) {
                reporter.printFatalErrorAndExit(
                    path + ": No such file or directory");
            }
        }

        return paths;
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
                reporter.printFatalErrorAndExit(
                    "Output location '" + root + "' does not exist.");
            }
            if (!Files.isDirectory(root)) {
                reporter.printFatalErrorAndExit(
                    "Output location '" + root + "' is not a directory.");
            }
        }

        return root;
    }

    /**
     * If some errors were collected, print them and abort execution.
     * Otherwise, return.
     */
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

        List<Issue> issues = this.validator.validate(
                resource, CheckMode.ALL, CancelIndicator.NullImpl);

        for (Issue issue : issues) {
            // Issues may also relate to imported resources.
            URI uri = issue.getUriToProblem(); 
            try {
                issueCollector.accept(
                        new LfIssue(
                            issue.getMessage(),
                            issue.getSeverity(),
                            issue.getLineNumber(),
                            issue.getColumn(),
                            issue.getLineNumberEnd(),
                            issue.getColumnEnd(),
                            issue.getLength(),
                            FileUtil.toPath(uri)));
            } catch (IOException e) {
                reporter.printError(
                        "Unable to convert '" + uri + "' to path." + e);
            }
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
