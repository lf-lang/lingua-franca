package org.lflang.cli;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;

import picocli.CommandLine;
import picocli.CommandLine.ArgGroup;
import picocli.CommandLine.Command;
import picocli.CommandLine.Model.CommandSpec;

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
import com.google.gson.*;
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
    static class MutuallyExclusive {
        @Parameters(
        arity = "1..",
        paramLabel = "FILES",
        description = "Paths of the files to run Lingua Franca programs on.")
            protected List<Path> files;

        @Option(
        names="--json",
        description="JSON object containing CLI arguments.")
            private String jsonString;

        @Option(
        names="--json-file",
        description="JSON file containing CLI arguments.")
            private Path jsonFile;
    }

    @ArgGroup(exclusive = true, multiplicity = "1")
    MutuallyExclusive topLevelArg;

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
     */ 
    public void run() {

        // If args are given in a json file, store its contents in jsonString.
        if (topLevelArg.jsonFile != null) {
            topLevelArg.jsonString = new String(
                    Files.readAllBytes(topLevelArg.jsonFile));
        }

        // If args are given in a json string, (1) unpack them into an args
        // string, and (2) call cmd.execute on them, which assigns them to their
        // correct instance variables, then (3) recurses into run().
        if (topLevelArg.jsonString != null) {
            // TODO: error handling.
            String args = jsonStringToArgs(topLevelArg.jsonString);
            // Execute application on unpacked args.
            CommandLine cmd = spec.commandLine();
            int exitCode = cmd.execute(args);
            io.callSystemExit(exitCode);

        // Args are already unpacked; invoke tool-specific logic.
        } else {
            runTool();
        }
    }

    /*
     * The entrypoint of tool-specific logic.
     * Lfc and Lff have their own specific implementations for this method.
     */
    public abstract void runTool();

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
        List<Path> paths = topLevelArg.files.stream()
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

    /**
     * Constructs an arguments string (specific to lingua franca cli tools) from 
     * a json string. 
     *
     * The given json object takes the following form:
     * {
     *      "src": "/home/lf-user/workspace/lf-test/src/main.lf",
     *      "out": "/home/lf-user/workspace/lf-test/src-gen",
     *      "properties": {
     *          "fast": true,
     *          "federated": true
     *      }
     * }
     */
    private String jsonStringToArgs(String jsonString) {
        String args = "";
        // Get top-level json object.
        JsonObject jsonObject = JsonParser
            .parseString(jsonString)
            .getAsJsonObject();

        // Append input and output paths.
        args += jsonObject.get("src").getAsString();
        args += " --output-path " + jsonObject.get("out").getAsString();

        // Get the remaining properties.
        Set<Entry<String, JsonElement>> entrySet = jsonObject
            .getAsJsonObject("properties")
            .entrySet();

        // Append the remaining properties to the args string.
        for(Entry<String,JsonElement> entry : entrySet) {
            String property = entry.getKey();
            String value = entry.getValue().getAsString();

            // Boolean except threading.
            if (value == "true" && property != "threading") {
                args += " --" + property;
                // Options with arguments.
            } else {
                args += String.format(" --%1$s %2$s", property, value);
            }
        }
        return args;
    }
}
