package org.lflang.cli;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Properties;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.util.FileUtil;

import com.google.inject.Inject;
import com.google.inject.Provider;

/**
 * Base class for standalone CLI applications.
 *
 * @author {Marten Lohstroh <marten@berkeley.edu>}
 * @author {Christian Menard <christian.menard@tu-dresden.de>}
 * @author {Billy Bao <billybao@berkeley.edu>}
 */
public class CliBase {
    /**
     * Object for interpreting command line arguments.
     */
    protected CommandLine cmd;
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
     * Store command-line arguments as properties, to be passed on to the runtime.
     * @param passOptions Which options should be passed to the runtime.
     * @return Provided arguments in cmd as properties, which should be passed to the runtime.
     */
    protected Properties filterProps(List<Option> passOptions) {
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

    // visible in tests
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
    // visible in tests
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
    // visible in tests
    public Resource getResource(Path path) {
        final ResourceSet set = this.resourceSetProvider.get();
        try {
            return set.getResource(URI.createFileURI(path.toString()), true);
        } catch (RuntimeException e) {
            return null;
        }
    }
}
