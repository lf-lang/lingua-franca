package org.lflang.generator;

import java.util.LinkedHashSet;
import java.util.Set;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.MessageReporter;
import org.lflang.generator.LFGeneratorContext.Mode;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.KeepaliveProperty;

/**
 * A helper class with functions that may be useful for code generators. This is created to ease our
 * transition from Xtend and possibly Eclipse. All functions in this class should instead be in
 * GeneratorUtils.kt, but Eclipse cannot handle Kotlin files.
 */
public class GeneratorUtils {

  private GeneratorUtils() {
    // utility class
  }

  /** Return the target declaration found in the given resource. */
  public static TargetDecl findTargetDecl(Resource resource) {
    return findAll(resource, TargetDecl.class).iterator().next();
  }

  /**
   * Look for physical actions in 'resource'. If appropriate, set keepalive to true in {@code
   * targetConfig}. This is a helper function for setTargetConfig. It should not be used elsewhere.
   */
  public static void accommodatePhysicalActionsIfPresent(
      Set<Resource> resources,
      boolean setsKeepAliveOptionAutomatically,
      TargetConfig targetConfig,
      MessageReporter messageReporter) {
    if (!setsKeepAliveOptionAutomatically) {
      return;
    }
    for (Resource resource : resources) {
      for (Action action : findAll(resource, Action.class)) {
        if (action.getOrigin() == ActionOrigin.PHYSICAL
            && !targetConfig.isSet(KeepaliveProperty.INSTANCE)
            && !targetConfig.get(KeepaliveProperty.INSTANCE)) {
          // Keepalive was explicitly set to false; set it to true.

          KeepaliveProperty.INSTANCE.override(targetConfig, true);
          String message =
              String.format(
                  "Setting %s to true because of the physical action %s.",
                  KeepaliveProperty.INSTANCE.name(), action.getName());
          messageReporter.at(action).warning(message);
          return;
        }
      }
    }
  }

  /**
   * Return all instances of {@code eObjectType} in {@code resource}.
   *
   * @param resource A resource to be searched.
   * @param nodeType The type of the desired parse tree nodes.
   * @param <T> The type of the desired parse tree nodes.
   * @return all instances of {@code eObjectType} in {@code resource}
   */
  public static <T> Iterable<T> findAll(Resource resource, Class<T> nodeType) {
    return () -> IteratorExtensions.filter(resource.getAllContents(), nodeType);
  }

  /**
   * Return the resources that provide the given reactors and their ancestors.
   *
   * @param reactors The reactors for which to find containing resources.
   * @return the resources that provide the given reactors.
   */
  public static Set<Resource> getResources(Iterable<Reactor> reactors) {
    Set<Resource> visited = new LinkedHashSet<>();
    for (Reactor r : reactors) {
      Resource resource = r.eResource();
      if (!visited.contains(resource)) {
        addInheritedResources(r, visited);
      }
    }
    return visited;
  }

  /** Collect all resources associated with reactor through class inheritance. */
  private static void addInheritedResources(Reactor reactor, Set<Resource> resources) {
    resources.add(reactor.eResource());
    for (var s : reactor.getSuperClasses()) {
      if (!resources.contains(s)) {
        if (s instanceof ImportedReactor i) {
          addInheritedResources(i.getReactorClass(), resources);
        } else if (s instanceof Reactor r) {
          addInheritedResources(r, resources);
        }
      }
    }
  }

  /**
   * If the mode is Mode.EPOCH (the code generator is running in an Eclipse IDE), then refresh the
   * project. This will ensure that any generated files become visible in the project.
   *
   * @param resource The resource.
   * @param compilerMode An indicator of whether Epoch is running.
   */
  public static void refreshProject(Resource resource, Mode compilerMode) {
    if (compilerMode == LFGeneratorContext.Mode.EPOCH) {
      URI uri = resource.getURI();
      if (uri.isPlatformResource()) { // This condition should normally be met when running Epoch
        IResource member =
            ResourcesPlugin.getWorkspace().getRoot().findMember(uri.toPlatformString(true));
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
   * Check whether code can be generated; report any problems and inform the context accordingly.
   *
   * @return Whether it is possible to generate code.
   */
  public static boolean canGenerate(
      Boolean errorsOccurred,
      Instantiation mainDef,
      MessageReporter messageReporter,
      LFGeneratorContext context) {
    // stop if there are any errors found in the program by doGenerate() in GeneratorBase
    if (errorsOccurred) {
      context.finish(GeneratorResult.FAILED);
      return false;
    }
    // abort if there is no main reactor
    if (mainDef == null) {
      messageReporter
          .nowhere()
          .info(
              "The given Lingua Franca program does not define a main reactor. Therefore, no code"
                  + " was generated.");
      context.finish(GeneratorResult.NOTHING);
      return false;
    }
    return true;
  }
}
