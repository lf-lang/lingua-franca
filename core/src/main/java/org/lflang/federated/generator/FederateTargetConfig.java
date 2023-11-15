package org.lflang.federated.generator;

import static org.lflang.ast.ASTUtils.convertToEmptyListIfNull;

import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.KeyValuePair;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.FileListProperty;
import org.lflang.util.FileUtil;

/**
 * Subclass of TargetConfig with a specialized constructor for creating configurations for
 * federates.
 *
 * @author Marten Lohstroh
 */
public class FederateTargetConfig extends TargetConfig {

  /**
   * Create a configuration for a federate given a main context and the resource in which the class
   * of the federate is specified.
   *
   * @param context The generator context.
   * @param federateResource The resource in which to find the reactor class of the federate.
   */
  public FederateTargetConfig(LFGeneratorContext context, Resource federateResource) {
    // Create target config with the target based on the federate (not the main resource).
    super(Target.fromDecl(GeneratorUtils.findTargetDecl(federateResource)));
    var federationResource = context.getFileConfig().resource;
    var reporter = context.getErrorReporter();

    this.mainResource = federationResource;

    // Load properties from the main file
    load(federationResource, reporter);

    // Load properties from the federate file
    mergeImportedConfig(federateResource, federationResource, reporter);

    // Load properties from the generator context
    load(context.getArgs(), reporter);

    clearPropertiesToIgnore();

    ((FederationFileConfig) context.getFileConfig()).relativizePaths(this);

    this.validate(reporter);
  }

  /**
   * If the federate that target configuration applies to is imported, merge target properties
   * declared in the file that it was imported from.
   *
   * @param federateResource The resource where the class of the federate is specified.
   * @param mainResource The resource in which the federation (i.e., main reactor) is specified.
   * @param messageReporter An error reporter to use when problems are encountered.
   */
  private void mergeImportedConfig(
      Resource federateResource, Resource mainResource, MessageReporter messageReporter) {
    // If the federate is imported, then update the configuration based on target properties
    // in the imported file.
    if (!federateResource.equals(mainResource)) {
      var importedTargetDecl = GeneratorUtils.findTargetDecl(federateResource);
      var targetProperties = importedTargetDecl.getConfig();
      if (targetProperties != null) {
        // Merge properties
        update(
            this,
            convertToEmptyListIfNull(targetProperties.getPairs()),
            getRelativePath(mainResource, federateResource),
            messageReporter);
      }
    }
  }

  private Path getRelativePath(Resource source, Resource target) {
    return FileUtil.toPath(source.getURI())
        .getParent()
        .relativize(FileUtil.toPath(target.getURI()).getParent());
  }

  /** Method for the removal of things that should not appear in the target config of a federate. */
  private void clearPropertiesToIgnore() {
    this.reset(ClockSyncModeProperty.INSTANCE);
    this.reset(ClockSyncOptionsProperty.INSTANCE);
  }

  /**
   * Update the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param pairs AST node that holds all the target properties.
   * @param relativePath The path from the main resource to the resource from which the new
   *     properties originate.
   */
  public void update(
      TargetConfig config, List<KeyValuePair> pairs, Path relativePath, MessageReporter err) {
    pairs.forEach(
        pair -> {
          var p = config.forName(pair.getName());
          if (p.isPresent()) {
            var value = pair.getValue();
            var property = p.get();
            if (property instanceof FileListProperty fileListProperty) {
              var files =
                  ASTUtils.elementToListOfStrings(value).stream()
                      .map(relativePath::resolve) // assume all paths are relative
                      .map(Objects::toString)
                      .toList();
              fileListProperty.update(config, files);
            } else {
              p.get().update(this, pair, err);
            }
          }
        });
  }
}
