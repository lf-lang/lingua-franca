package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.eclipse.emf.ecore.util.EcoreUtil;

import org.lflang.ASTUtils;
import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
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

        federates.forEach(federate -> generateCausalityInterfaces(federate, interfacesFolderPath));
    }

    /**
     * Generate a causality interface for {@code federate} and put it in
     * `include/$federate.name$_interface.lf` in the fed-gen directory.
     */
    private void generateCausalityInterfaces(FederateInstance federate, Path interfacesFolderPath) {
        var renderer = FormattingUtils.renderer(federate.target);

        Path interfaceFilePath = interfacesFolderPath.resolve(federate.name + "_interface.lf");

        CodeBuilder code = new CodeBuilder();
        Reactor reactor = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
        TargetDecl interfaceTarget = ASTUtils.factory.createTargetDecl();
        interfaceTarget.setName(federate.target.getName()); // FIXME: This will interfere with mixed-target federations
        code.pr(renderer.apply(interfaceTarget));
        code.pr(renderer.apply(stripReactorForZeroDelayInterface(reactor)));

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

        // Replace reactor classes of instantiations with their stripped-down versions.
        reactorToReturn.getInstantiations().forEach(instantiation -> {
            instantiation.setReactorClass(
                stripReactorForZeroDelayInterface(ASTUtils.toDefinition(instantiation.getReactorClass()))
            );
        });
        return reactorToReturn;
    }
}
