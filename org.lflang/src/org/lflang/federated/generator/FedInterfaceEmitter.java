package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;

import org.lflang.ASTUtils;
import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorUtils;
import org.lflang.lf.Action;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.lf.Timer;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * Helper class that generates a stripped-down version of each federate to act
 * as a causality interface in other federates.
 *
 * @author Edward A. Lee (eal@berkeley.edu)
 * @author Soroush Bateni (soroush@berkeley.edu)
 */
public class FedInterfaceEmitter {

    /**
     * Generate stripped-down versions of each federate that can act as the interface
     * for that federate. These will be put in the {@code FedFileConfig#getFedSrcPath()}/include
     * folder.
     *
     * @param federates A list of federates.
     * @param fileConfig Used to create a file for each interface.
     * @throws IOException
     */
    public void generateInterfacesForFederates(List<FederateInstance> federates, FedFileConfig fileConfig) throws IOException {
        var interfacesFolderPath = fileConfig.getFedSrcPath().resolve(
            "include/interfaces/"
        );
        Files.createDirectories(interfacesFolderPath);

        List<Reactor> visitedReactorClasses = new ArrayList<>();

        federates.forEach(federate -> generateCausalityInterfaces(federate, interfacesFolderPath, visitedReactorClasses));
    }

    /**
     * Generate a causality interface for {@code federate} and all its contained reactors.
     */
    private void generateCausalityInterfaces
    (FederateInstance federate,
     Path interfacesFolderPath,
     List<Reactor> visitedReactorClasses
    ) {
        var renderer = FormattingUtils.renderer(federate.target);

        generateZeroDelayCausalityInterfaceForReactor(
            ASTUtils.toDefinition(federate.instantiation.getReactorClass()),
            interfacesFolderPath,
            visitedReactorClasses,
            renderer
        );
    }

    /**
     * Generate a causality interface for {@code reactor} that preserves zero-delay
     * connections from input ports to output ports within that reactor.
     *
     * Will recursively generate causality interfaces for any contained reactors.
     * @param renderer Used to render each reactor
     */
    private void generateZeroDelayCausalityInterfaceForReactor(
        Reactor reactor,
        Path interfacesFolderPath,
        List<Reactor> visitedReactorClasses,
        Function<EObject,
            String> renderer
    ) {
        if(visitedReactorClasses.contains(reactor)) return;
        visitedReactorClasses.add(reactor);
        CodeBuilder code = new CodeBuilder();
        Path interfaceFilePath = interfacesFolderPath.resolve(reactor.getName() + "_interface.lf");
        TargetDecl interfaceTarget = ASTUtils.factory.createTargetDecl();
        interfaceTarget.setName(GeneratorUtils.findTarget(reactor.eResource()).getName()); // FIXME: This will interfere with mixed-target federations
        code.pr(renderer.apply(interfaceTarget));
        code.pr(renderer.apply(stripReactorForZeroDelayInterface(reactor)));


        reactor.getInstantiations().forEach(instantiation -> {
            var containedReactor = ASTUtils.toDefinition(instantiation.getReactorClass());
            Path containedReactorInterfacePath = interfacesFolderPath.resolve(containedReactor.getName() + "_interface.lf");
            // Import the interface for the contained reactor class
            code.pr("import "+containedReactor.getName()+" from \""+containedReactorInterfacePath+"\"");
            // Generate interfaces for reactor classes of contained reactors
            generateZeroDelayCausalityInterfaceForReactor(
                containedReactor,
                interfacesFolderPath,
                visitedReactorClasses,
                renderer
            );
        });

        // Write the federate's interface to file
        try (var interfaceWriter = Files.newBufferedWriter(interfaceFilePath)) {
            interfaceWriter.write(code.getCode());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // Possible solution: keep all federates because of the intricacies of dealing with connections that involve banks
    // TODO: Generate instantiations
    // TODO: Keep connections

    /**
     * Strip out all unrelated elements of a reactor to create a zero-delay causality
     * interface of that reactor.
     *
     * FIXME: Keep track of already converted reactor classes so we don't have to copy them.
     */
    private Reactor stripReactorForZeroDelayInterface(Reactor reactor) {
        Reactor reactorToReturn = EcoreUtil.copy(reactor);
        reactorToReturn.getReactions().stream().forEach(
            reaction -> {
                // Remove timers and action triggers (to be replaced by startup).
                var triggersRemoved = reaction.getTriggers().removeIf(trigger -> {
                    if (trigger instanceof VarRef) {
                        Variable triggerVar = ((VarRef)trigger).getVariable();
                        if (triggerVar instanceof Action || triggerVar instanceof Timer) {
                            return true;
                        }
                    }
                    return false;
                });
                if (triggersRemoved) {
                    // At least one trigger was removed because it was an action or a timer.
                    // Add startup to the triggers of this reaction.
                    BuiltinTriggerRef startup = ASTUtils.factory.createBuiltinTriggerRef();
                    startup.setType(BuiltinTrigger.STARTUP);
                    reaction.getTriggers().add(startup);
                }
                // Remove action sources
                reaction.getSources().removeIf(source -> source.getVariable() instanceof Action);
                // Remove action effects
                reaction.getEffects().removeIf(effect -> effect.getVariable() instanceof Action);
                reaction.getCode().setBody("");
            });

        // Remove timers, actions, parameters, and state variables
        reactorToReturn.getTimers().clear();
        reactorToReturn.getActions().clear();
        reactorToReturn.getParameters().clear();
        reactorToReturn.getStateVars().clear();


        reactorToReturn.getInputs().forEach(input -> {
            // Remove all multiports and replace them with ordinary ports since we only
            // need high-level dependency information. FIXME: This is most likely incorrect
            input.setWidthSpec(null);
            // Convert port types to time so that they become target-independent
            if (input.getType() != null) {
                var type = ASTUtils.factory.createType();
                type.setTime(true);
                input.setType(type);
            }
        });
        reactorToReturn.getOutputs().forEach(output -> {
            // Remove all multiports and replace them with ordinary ports since we only
            // need high-level dependency information. FIXME: This is most likely incorrect
            output.setWidthSpec(null);
            // Convert port types to time so that they become target-independent
            if (output.getType() != null) {
                var type = ASTUtils.factory.createType();
                type.setTime(true);
                output.setType(type);
            }
        });

        return reactorToReturn;
    }
}
