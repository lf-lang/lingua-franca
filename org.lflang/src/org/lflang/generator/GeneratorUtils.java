package org.lflang.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Set;

import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.diagnostics.Severity;
import org.eclipse.xtext.generator.IGeneratorContext;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.validation.CheckMode;
import org.eclipse.xtext.validation.IResourceValidator;
import org.eclipse.xtext.validation.Issue;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetConfig.DockerOptions;
import org.lflang.TargetProperty.BuildType;
import org.lflang.TargetProperty.LogLevel;
import org.lflang.TargetProperty.UnionType;
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
import org.lflang.lf.Reaction;
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
     * @param args The commandline arguments to process.
     * @param target The target properties AST node.
     * @param errorReporter The error reporter to which errors should be sent.
     */
    public static TargetConfig getTargetConfig(
        Properties args,
        TargetDecl target,
        ErrorReporter errorReporter
    ) {
        final TargetConfig targetConfig = new TargetConfig(target); // FIXME: why not just do all of this in the constructor?
        if (target.getConfig() != null) {
            List<KeyValuePair> pairs = target.getConfig().getPairs();
            TargetProperty.set(targetConfig, pairs != null ? pairs : List.of(), errorReporter);
        }
        if (args.containsKey("no-compile")) {
            targetConfig.noCompile = true;
        }
        if (args.containsKey("docker")) {
            var arg = args.getProperty("docker");
            if (Boolean.parseBoolean(arg)) {
                targetConfig.dockerOptions = new DockerOptions();
            } else {
                targetConfig.dockerOptions = null;
            }
            // FIXME: this is pretty ad-hoc and does not account for more complex overrides yet.
        }
        if (args.containsKey("build-type")) {
            targetConfig.cmakeBuildType = (BuildType) UnionType.BUILD_TYPE_UNION.forName(args.getProperty("build-type"));
        }
        if (args.containsKey("logging")) {
            targetConfig.logLevel = LogLevel.valueOf(args.getProperty("logging").toUpperCase());
        }
        if (args.containsKey("workers")) {
            targetConfig.workers = Integer.parseInt(args.getProperty("workers"));
        }
        if (args.containsKey("threading")) {
            targetConfig.threading = Boolean.parseBoolean(args.getProperty("threading"));
        }
        if (args.containsKey("target-compiler")) {
            targetConfig.compiler = args.getProperty("target-compiler");
        }
        if (args.containsKey("scheduler")) {
            targetConfig.schedulerType = SchedulerOption.valueOf(
                args.getProperty("scheduler")
            );
            targetConfig.setByUser.add(TargetProperty.SCHEDULER);
        }
        if (args.containsKey("target-flags")) {
            targetConfig.compilerFlags.clear();
            if (!args.getProperty("target-flags").isEmpty()) {
                targetConfig.compilerFlags.addAll(List.of(
                    args.getProperty("target-flags").split(" ")
                ));
            }
        }
        if (args.containsKey("runtime-version")) {
            targetConfig.runtimeVersion = args.getProperty("runtime-version");
        }
        if (args.containsKey("external-runtime-path")) {
            targetConfig.externalRuntimePath = args.getProperty("external-runtime-path");
        }
        if (args.containsKey(TargetProperty.KEEPALIVE.description)) {
            targetConfig.keepalive = Boolean.parseBoolean(
                args.getProperty(TargetProperty.KEEPALIVE.description));
        }
        return targetConfig;
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
        var target = ASTUtils.targetDecl(resource);
        KeyValuePairs config = target.getConfig();
        var targetConfig = new TargetConfig(target);
        if (config != null) {
            List<KeyValuePair> pairs = config.getPairs();
            TargetProperty.set(targetConfig, pairs != null ? pairs : List.of(), errorReporter);
        }
        FileConfig fc = LFGenerator.createFileConfig(resource, srcGenBasePath, context.getFileConfig().useHierarchicalBin);
        return new LFResource(resource, fc, targetConfig);
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
            errorReporter.reportInfo("INFO: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.");
            context.finish(GeneratorResult.NOTHING);
            return false;
        }
        return true;
    }
}
