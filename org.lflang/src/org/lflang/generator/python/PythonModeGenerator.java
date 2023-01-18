package org.lflang.generator.python;

import java.util.List;

import org.eclipse.emf.ecore.util.EcoreUtil;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;

/**
 * Helper class to handle modes in Python programs.
 *
 * @author Soroush Bateni
 *
 */
public class PythonModeGenerator {
    /**
     * Generate reset reactions in modes to reset state variables.
     *
     * @param reactors A list of reactors in the program, some of which could contain modes.
     */
    public static void generateResetReactionsIfNeeded(List<Reactor> reactors) {
        for (Reactor reactor : reactors) {
            generateStartupReactionsInReactor(reactor);
        }
    }

    /**
     * Generate reset reactions that reset state variables in
     * - the reactor, and,
     * - the modes within the reactor.
     *
     * @param reactor The reactor.
     */
    private static void generateStartupReactionsInReactor(Reactor reactor) {

        // Create a reaction with a reset trigger
        BuiltinTriggerRef resetTrigger = LfFactory.eINSTANCE.createBuiltinTriggerRef();
        resetTrigger.setType(BuiltinTrigger.RESET);
        Reaction baseReaction = LfFactory.eINSTANCE.createReaction();
        baseReaction.getTriggers().add(resetTrigger);

        if (!reactor.getStateVars().isEmpty() && reactor.getStateVars().stream().anyMatch(s -> s.isReset())) {
            // Create a reaction body that resets all state variables (that are not in a mode)
            // to their initial value.
            var reactionBody = LfFactory.eINSTANCE.createCode();
            CodeBuilder code = new CodeBuilder();
            code.pr("# Reset the following state variables to their initial value.");
            for (var state: reactor.getStateVars()) {
                if (state.isReset()) {
                    code.pr("self."+state.getName()+" = "+ PythonStateGenerator.generatePythonInitializer(state));
                }
            }
            reactionBody.setBody(code.toString());
            baseReaction.setCode(reactionBody);

            reactor.getReactions().add(0, baseReaction);
        }


        var reactorModes = reactor.getModes();
        if (!reactorModes.isEmpty()) {
            for (Mode mode : reactorModes) {
                if (mode.getStateVars().isEmpty() || mode.getStateVars().stream().allMatch(s -> !s.isReset())) {
                    continue;
                }
                Reaction reaction = EcoreUtil.copy(baseReaction);

                // Create a reaction body that resets all state variables to their initial value.
                var reactionBody = LfFactory.eINSTANCE.createCode();
                CodeBuilder code = new CodeBuilder();
                code.pr("# Reset the following state variables to their initial value.");
                for (var state: mode.getStateVars()) {
                    if (state.isReset()) {
                        code.pr("self."+state.getName()+" = "+ PythonStateGenerator.generatePythonInitializer(state));
                    }
                }
                reactionBody.setBody(code.toString());
                reaction.setCode(reactionBody);

                try {
                    mode.getReactions().add(0, reaction);
                } catch (IndexOutOfBoundsException e) {
                    // There are no scoping for state variables.
                    // We add this reaction for now so that it
                    // still resets state variables even if there
                    // are no reactions in this mode.
                    mode.getReactions().add(reaction);
                }

            }
        }
    }
}
