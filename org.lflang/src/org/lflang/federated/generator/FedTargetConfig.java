package org.lflang.federated.generator;

import static org.lflang.ASTUtils.convertToEmptyListIfNull;

import org.lflang.ErrorReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.eclipse.emf.ecore.resource.Resource;

/**
 * Subclass of TargetConfig with a specialized constructor for creating configurations for federates.
 * @author Marten Lohstroh
 */
public class FedTargetConfig extends TargetConfig {

    /**
     * Create a configuration for a federate given a main context and the resource in which the class
     * of the federate is specified.
     * @param context The generator context.
     * @param federateResource The resource in which to find the reactor class of the federate.
     */
    public FedTargetConfig(LFGeneratorContext context, Resource federateResource) {
        // Create target config based on the main .lf file
        super(
            context.getArgs(),
            GeneratorUtils.findTargetDecl(context.getFileConfig().resource),
            context.getErrorReporter()
        );

        mergeImportedConfig(
            federateResource,
            context.getFileConfig().resource,
            context.getErrorReporter()
        );

        clearPropertiesToIgnore();

        ((FedFileConfig)context.getFileConfig()).relativizePaths(this);

    }

    /**
     * If the federate that target configuration applies to is imported, merge target properties
     * declared in the file that it was imported from.
     * @param federateResource The resource where the class of the federate is specified.
     * @param mainResource The resource in which the federation (i.e., main reactor) is specified.
     * @param errorReporter An error reporter to use when problems are encountered.
     */
    private void mergeImportedConfig(
        Resource federateResource,
        Resource mainResource,
        ErrorReporter errorReporter) {
        // If the federate is imported, then update the configuration based on target properties
        // in the imported file.
        if (!federateResource.equals(mainResource)) {
            var importedTargetDecl = GeneratorUtils.findTargetDecl(federateResource);
            // Merge properties
            TargetProperty.update(
                this,
                convertToEmptyListIfNull(importedTargetDecl.getConfig().getPairs()),
                errorReporter
            );
        }
    }

    /**
     * Method for the removal of things that should not appear in the target config of a federate.
     */
    private void clearPropertiesToIgnore() {
        this.setByUser.remove(TargetProperty.CLOCK_SYNC);
        this.setByUser.remove(TargetProperty.CLOCK_SYNC_OPTIONS);
    }
}
