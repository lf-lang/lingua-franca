package org.lflang.generator.python;

import java.util.List;

import org.eclipse.emf.ecore.util.EcoreUtil;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.TriggerRef;

/**
 * Helper class to handle modes in Python programs.
 * 
 * @author{Soroush Bateni <soroush@utdallas.edu>}
 *
 */
public class PythonModeGenerator {
    /**
     * Generate startup reactions in modes.
     * 
     * Startup reactions (reactions that have startup in their list of triggers)
     * will be triggered when the mode is entered for the first time and on each subsequent
     * reset transition to that mode. These reactions could be useful for targets
     * to perform cleanups, for example, to reset state variables.
     * 
     * @param reactors A list of reactors in the program, some of which could contain modes.
     */
    public static void generateStartupReactionsInModesIfNeeded(List<Reactor> reactors) {
        for (Reactor reactor : reactors) {
            generateStartupReactionsInReactor(reactor);
        }
    }
    
    /**
     * Generate startup reactions that reset state variables in 
     * - the reactor, and,
     * - the modes within the reactor.
     * 
     * @param reactor The reactor.
     */
    private static void generateStartupReactionsInReactor(Reactor reactor) {
        
        // Create a reaction with a startup trigger
        TriggerRef startupTrigger = LfFactory.eINSTANCE.createTriggerRef();
        startupTrigger.setStartup(true);
        Reaction baseReaction = LfFactory.eINSTANCE.createReaction();
        baseReaction.getTriggers().add(startupTrigger);
        
        if (!reactor.getStateVars().isEmpty()) {
            // Create a reaction body that resets all state variables (that are not in a mode)
            // to their initial value.
            var reactionBody = LfFactory.eINSTANCE.createCode();
            CodeBuilder code = new CodeBuilder();
            code.pr("# Reset the following state variables to their initial value.");
            for (var state: reactor.getStateVars()) {
                code.pr("self."+state.getName()+" = "+PythonStateGenerator.generatePythonInitializer(state));
            }
            reactionBody.setBody(code.toString());
            baseReaction.setCode(reactionBody);
            
            reactor.getReactions().add(0, baseReaction);
        }
            
        
        var reactorModes = reactor.getModes();
        if (!reactorModes.isEmpty()) {
            for (Mode mode : reactorModes) {
                if (mode.getStateVars().isEmpty()) {
                    continue;
                }
                Reaction reaction = EcoreUtil.copy(baseReaction);
                
                // Create a reaction body that resets all state variables to their initial value.
                var reactionBody = LfFactory.eINSTANCE.createCode();
                CodeBuilder code = new CodeBuilder();
                code.pr("# Reset the following state variables to their initial value.");
                for (var state: mode.getStateVars()) {
                    code.pr("self."+state.getName()+" = "+PythonStateGenerator.generatePythonInitializer(state));
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
