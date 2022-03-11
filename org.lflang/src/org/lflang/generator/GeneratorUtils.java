package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.generator.LFGeneratorContext.Mode;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.SchedulerOption;
import org.lflang.graph.InstantiationGraph;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Import;
import org.lflang.lf.Instantiation;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.util.FileUtil;

/**
 * A helper class with functions that may be useful for code
 * generators.
 * This is created to ease our transition from Xtend and
 * possibly Eclipse. All functions in this class should
 * instead be in GeneratorUtils.kt, but Eclipse cannot
 * handle Kotlin files.
 */
public class GeneratorUtils {

    private GeneratorUtils() {
        // utility class
    }

    /**
     * Return the target declaration found in the given resource.
     */
    public static TargetDecl findTarget(Resource resource) {
        return findAll(resource, TargetDecl.class).iterator().next();
    }

    /**
     * Set the appropriate target properties based on the target properties of
     * the main .lf file and the given command-line arguments, if applicable.
     * @param context The generator invocation context.
     * @param target The target configuration that appears in an LF source file.
     * @param targetConfig The target config to be updated.
     * @param errorReporter The error reporter to which errors should be sent.
     */
    public static void setTargetConfig(
        LFGeneratorContext context,
        TargetDecl target,
        TargetConfig targetConfig,
        ErrorReporter errorReporter
    ) {
        if (target.getConfig() != null) {
            List<KeyValuePair> pairs = target.getConfig().getPairs();
            TargetProperty.set(targetConfig, pairs != null ? pairs : List.of(), errorReporter);
        }
        if (context.getArgs().containsKey("no-compile")) {
            targetConfig.noCompile = true;
        }
        if (context.getArgs().containsKey("workers")) {
            targetConfig.workers = Integer.parseInt(context.getArgs().getProperty("workers"));
        }
        if (context.getArgs().containsKey("threading")) {
            targetConfig.threading = Boolean.parseBoolean(context.getArgs().getProperty("threading"));
        }
        if (context.getArgs().containsKey("target-compiler")) {
            targetConfig.compiler = context.getArgs().getProperty("target-compiler");
        }
        if (context.getArgs().containsKey("scheduler")) {
            targetConfig.schedulerType = SchedulerOption.valueOf(
                context.getArgs().getProperty("scheduler")
            );
            targetConfig.setByUser.add(TargetProperty.SCHEDULER);
        }
        if (context.getArgs().containsKey("target-flags")) {
            targetConfig.compilerFlags.clear();
            if (!context.getArgs().getProperty("target-flags").isEmpty()) {
                targetConfig.compilerFlags.addAll(List.of(
                    context.getArgs().getProperty("target-flags").split(" ")
                ));
            }
        }
        if (context.getArgs().containsKey("runtime-version")) {
            targetConfig.runtimeVersion = context.getArgs().getProperty("runtime-version");
        }
        if (context.getArgs().containsKey("external-runtime-path")) {
            targetConfig.externalRuntimePath = context.getArgs().getProperty("external-runtime-path");
        }
        if (context.getArgs().containsKey(TargetProperty.KEEPALIVE.description)) {
            targetConfig.keepalive = Boolean.parseBoolean(
                context.getArgs().getProperty(TargetProperty.KEEPALIVE.description));
        }
    }

    /**
     * Look for physical actions in 'resource'.
     * If appropriate, set keepalive to true in
     * {@code targetConfig}.
     * This is a helper function for setTargetConfig. It
     * should not be used elsewhere.
     */
    public static void accommodatePhysicalActionsIfPresent(
        List<Resource> resources,
        boolean setsKeepAliveOptionAutomatically,
        TargetConfig targetConfig,
        ErrorReporter errorReporter
    ) {
        if (!setsKeepAliveOptionAutomatically) {
            return;
        }
        for (Resource resource : resources) {
            for (Action action : findAll(resource, Action.class)) {
                if (action.getOrigin() == ActionOrigin.PHYSICAL && 
                    // Check if the user has explicitly set keepalive to false
                    !targetConfig.setByUser.contains(TargetProperty.KEEPALIVE) && 
                    !targetConfig.keepalive
                ) {
                    // If not, set it to true
                    targetConfig.keepalive = true;
                    errorReporter.reportWarning(
                        action,
                        String.format(
                            "Setting %s to true because of the physical action %s.",
                            TargetProperty.KEEPALIVE.getDisplayName(),
                            action.getName()
                        )
                    );
                    return;
                }
            }
        }
    }

    /**
     * Return all instances of {@code eObjectType} in
     * {@code resource}.
     * @param resource A resource to be searched.
     * @param nodeType The type of the desired parse tree
     *                    nodes.
     * @param <T> The type of the desired parse tree nodes.
     * @return all instances of {@code eObjectType} in
     * {@code resource}
     */
    public static <T> Iterable<T> findAll(Resource resource, Class<T> nodeType) {
        Iterator<EObject> contents = resource.getAllContents();
        assert contents != null : "Although getAllContents is not marked as NotNull, it should be.";
        EObject temp = null;
        while (!nodeType.isInstance(temp) && contents.hasNext()) temp = contents.next();
        EObject next_ = temp;
        return () -> new Iterator<>() {
            EObject next = next_;

            @Override
            public boolean hasNext() {
                return nodeType.isInstance(next);
            }

            @Override
            public T next() {
                // This cast is safe if hasNext() holds.
                assert hasNext() : "next() was called on an Iterator when hasNext() was false.";
                //noinspection unchecked
                T current = (T) next;
                next = null;
                while (!nodeType.isInstance(next) && contents.hasNext()) next = contents.next();
                return current;
            }
        };
    }

    /**
     * Validate the files containing reactors in the given
     * {@code instantiationGraph}. If a file is imported by
     * another file in the instantiation graph, propagate the
     * resulting errors to the importing file.
     * @param context The context providing the cancel
     *                indicator used by the validator.
     * @param fileConfig The file system configuration.
     * @param instantiationGraph A DAG containing all
     *                           reactors of interest.
     * @param errorReporter An error acceptor.
     */
    public static void validate(
        IGeneratorContext context,
        FileConfig fileConfig,
        InstantiationGraph instantiationGraph,
        ErrorReporter errorReporter
    ) {
        // NOTE: This method was previously misnamed validateImports.
        // It validates all files, including the main file that does the importing.
        // Also, it is now the only invocation of validation during code generation,
        // and yet it used to only report errors in the files doing the importing.
        IResourceValidator validator = ((XtextResource) fileConfig.resource).getResourceServiceProvider()
                                                                            .getResourceValidator();
        HashSet<Resource> bad = new HashSet<>();
        HashSet<Resource> visited = new HashSet<>();
        // The graph must be traversed in topological order so that errors will propagate through arbitrarily many
        // levels.
        for (Reactor reactor : instantiationGraph.nodesInTopologicalOrder()) {
            Resource resource = reactor.eResource();
            if (visited.contains(resource)) continue;
            visited.add(resource);
            List<Issue> issues = validator.validate(resource, CheckMode.ALL, context.getCancelIndicator());
            if (
                bad.contains(resource) || issues.size() > 0
            ) {
                // Report the error on this resource.
                Path path = null;
                try {
                    path = FileUtil.toPath(resource);
                } catch (IOException e) {
                    path = Paths.get("Unknown file"); // Not sure if this is what we want.
                }
                for (Issue issue : issues) {
                    errorReporter.reportError(path, issue.getLineNumber(), issue.getMessage());
                }
                
                // Report errors on resources that import this one.
                for (Reactor downstreamReactor : instantiationGraph.getDownstreamAdjacentNodes(reactor)) {
                    for (Import importStatement : ((Model) downstreamReactor.eContainer()).getImports()) {
                        // FIXME: This will report the error on ALL import statements in
                        // file doing the importing, not just the one importing this resource.
                        // I have no idea how to determine which import statement is the right one.
                        errorReporter.reportError(importStatement, String.format(
                            "Unresolved compilation issues in '%s': "
                                + issues.toString(), importStatement.getImportURI()
                        ));
                        bad.add(downstreamReactor.eResource());
                    }
                }
            }
        }
    }

    /**
     * Return the resources that provide the given
     * reactors.
     * @param reactors The reactors for which to find
     *                 containing resources.
     * @return the resources that provide the given
     * reactors.
     */
    public static List<Resource> getResources(Iterable<Reactor> reactors) {
        HashSet<Resource> visited = new HashSet<>();
        List<Resource> resources = new ArrayList<>();
        for (Reactor r : reactors) {
            Resource resource = r.eResource();
            if (!visited.contains(resource)) {
                visited.add(resource);
                resources.add(resource);
            }
        }
        return resources;
    }

    /**
     * Return the {@code LFResource} representation of the
     * given resource.
     * @param resource The {@code Resource} to be
     *                 represented as an {@code LFResource}
     * @param srcGenBasePath The root directory for any
     * generated sources associated with the resource.
     * @param context The generator invocation context.
     * @param errorReporter An error message acceptor.
     * @return the {@code LFResource} representation of the
     * given resource.
     */
    public static LFResource getLFResource(
        Resource resource,
        Path srcGenBasePath,
        LFGeneratorContext context,
        ErrorReporter errorReporter
    ) {
        TargetDecl target = GeneratorUtils.findTarget(resource);
        KeyValuePairs config = target.getConfig();
        var targetConfig = new TargetConfig();
        if (config != null) {
            List<KeyValuePair> pairs = config.getPairs();
            TargetProperty.set(targetConfig, pairs != null ? pairs : List.of(), errorReporter);
        }
        try {
            FileConfig fc = new FileConfig(resource, srcGenBasePath, context.useHierarchicalBin());
            return new LFResource(resource, fc, targetConfig);
        } catch (IOException e) {
            throw new RuntimeException("Failed to instantiate an imported resource because an I/O error "
                                           + "occurred.");
        }
    }

    /**
     * If the mode is Mode.EPOCH (the code generator is running in an
     * Eclipse IDE), then refresh the project. This will ensure that
     * any generated files become visible in the project.
     * @param resource The resource.
     * @param compilerMode An indicator of whether Epoch is running.
     */
    public static void refreshProject(Resource resource, Mode compilerMode) {
        if (compilerMode == LFGeneratorContext.Mode.EPOCH) {
            URI uri = resource.getURI();
            if (uri.isPlatformResource()) { // This condition should normally be met when running Epoch
                IResource member = ResourcesPlugin.getWorkspace().getRoot().findMember(uri.toPlatformString(true));
                try {
                    member.getProject().refreshLocal(IResource.DEPTH_INFINITE, null);
                } catch (CoreException e) {
                    System.err.println("Unable to refresh workspace: " + e);
                }
            }
        }
    }

    /** Return whether the operating system is Windows. */
    public static boolean isHostWindows() {
        return System.getProperty("os.name").toLowerCase().contains("win");
    }

    /**
     * Check whether code can be generated; report any problems
     * and inform the context accordingly.
     * @return Whether it is possible to generate code.
     */
    public static boolean canGenerate(
        Boolean errorsOccurred,
        Instantiation mainDef,
        ErrorReporter errorReporter,
        LFGeneratorContext context
    ) {
        // stop if there are any errors found in the program by doGenerate() in GeneratorBase
        if (errorsOccurred) {
            context.finish(GeneratorResult.FAILED);
            return false;
        }
        // abort if there is no main reactor
        if (mainDef == null) {
            errorReporter.reportWarning("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.");
            context.finish(GeneratorResult.NOTHING);
            return false;
        }
        return true;
    }
}
