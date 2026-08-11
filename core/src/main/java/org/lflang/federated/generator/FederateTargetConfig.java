package org.lflang.federated.generator;

import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;

/**
 * Subclass of TargetConfig with a specialized constructor for creating configurations for
 * federates.
 *
 * @author Marten Lohstroh
 * @ingroup Federated
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
    this(
        context,
        federateResource,
        Target.fromDecl(GeneratorUtils.findTargetDecl(federateResource)));
  }

  /**
   * Create a configuration for a federate with an explicitly supplied target, overriding the target
   * declared in the resource. Used by the Polyglot target, where the federation-level target is
   * {@code Polyglot} but each federate compiles with the target from its {@code @language}
   * annotation.
   *
   * @param context The generator context.
   * @param federateResource The resource in which to find the reactor class of the federate.
   * @param overrideTarget The actual compilation target to use for this federate.
   */
  public FederateTargetConfig(
      LFGeneratorContext context, Resource federateResource, Target overrideTarget) {
    super(overrideTarget);
    var federationResource = context.getFileConfig().resource;
    var reporter = context.getErrorReporter();

    this.mainResource = federationResource;

    // Load properties from the main file
    load(federationResource, reporter);

    // Load properties from the federate file
    mergeImportedConfig(federateResource, federationResource, p -> p.loadFromFederate(), reporter);

    // Load properties from the generator context
    load(context.getArgs(), reporter);

    ((FederationFileConfig) context.getFileConfig()).relativizePaths(this);

    this.validate(reporter);
  }
}
