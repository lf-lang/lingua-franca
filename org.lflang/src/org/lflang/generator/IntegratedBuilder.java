package org.lflang.generator;

import java.nio.file.Path;
import java.util.List;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.GeneratorDelegate;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.LanguageServerErrorReporter;

import com.google.inject.Inject;
import com.google.inject.Provider;

/**
 * Manages Lingua Franca code generation that is integrated
 * in an editor. FIXME: This should be more specific.
 */
public class IntegratedBuilder {

    /**
     * A <code>ReportMethod</code> is a way of reporting issues.
     */
    private interface ReportMethod {
        void apply(Path file, Integer line, String message);
    }

    /* ---------------------- INJECTED DEPENDENCIES ---------------------- */

    @Inject
    private IResourceValidator validator;
    @Inject
    private GeneratorDelegate generator;
    @Inject
    private JavaIoFileSystemAccess fileAccess;
    @Inject
    private Provider<ResourceSet> resourceSetProvider;

    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Generates code from the Lingua Franca file <code>f
     * </code>.
     * @param uri the URI of a Lingua Franca file
     */
    public void run(URI uri) {
        System.err.println("DEBUG: " + uri);
        // FIXME: A refactoring of the following line is needed. This refactor will affect FileConfig and
        //  org.lflang.lfc.Main.
        fileAccess.setOutputPath(FileConfig.findPackageRoot(Path.of(uri.path())).resolve(FileConfig.DEFAULT_SRC_GEN_DIR).toString());
        List<EObject> parseRoots = getResource(uri).getContents();
        if (parseRoots.isEmpty()) return;
        ErrorReporter errorReporter = new LanguageServerErrorReporter(parseRoots.get(0));
        validate(uri, errorReporter);
        if (!errorReporter.getErrorsOccurred()) doGenerate(uri);
    }

    /* ------------------------- PRIVATE METHODS ------------------------- */

    /**
     * Validates the Lingua Franca file <code>f</code>.
     * @param uri the URI of a Lingua Franca file
     * @param errorReporter the reporter with which to
     *                      report errors
     */
    private void validate(URI uri, ErrorReporter errorReporter) {
        for (Issue issue : validator.validate(getResource(uri), CheckMode.ALL, CancelIndicator.NullImpl)) {
            getReportMethod(errorReporter, issue.getSeverity()).apply(
                Path.of(uri.path()), issue.getLineNumber(), issue.getMessage()
            );
        }
    }

    /**
     * Generates code from the contents of <code>f</code>.
     * @param uri the URI of a Lingua Franca file
     */
    private void doGenerate(URI uri) {
        generator.generate(getResource(uri), fileAccess, new SlowIntegratedContext());
    }

    /**
     * Returns the resource corresponding to <code>uri
     * </code>.
     * @param uri the URI of a Lingua Franca file
     * @return the resource corresponding to <code>uri
     * </code>
     */
    private Resource getResource(URI uri) {
        return resourceSetProvider.get().getResource(uri, true);
    }

    /**
     * Returns the appropriate reporting method for the
     * given <code>Severity</code>.
     * @param severity an arbitrary <code>Severity</code>
     * @return the appropriate reporting method for
     * <code>severity</code>
     */
    private ReportMethod getReportMethod(ErrorReporter errorReporter, Severity severity) {
        if (severity == Severity.ERROR) return errorReporter::reportError;
        return errorReporter::reportWarning;
    }
}
