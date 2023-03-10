package org.lflang.cli;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;

import picocli.CommandLine;
import picocli.CommandLine.Option;
import picocli.CommandLine.Parameters;

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
import org.lflang.util.FileUtil;
import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;

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
     * Options and parameters present in both Lfc and Lff.
     */
    @Parameters(
        arity = "1..",
        paramLabel = "FILES",
        description = "Paths of the files to run Lingua Franca programs on.")
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

    /**
     * The entrypoint of Picocli applications - the first method called when 
     * CliBase, which implements the Runnable interface, is instantiated.
     * Lfc and Lff have their own specific implementations for this method.
     */ 
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
