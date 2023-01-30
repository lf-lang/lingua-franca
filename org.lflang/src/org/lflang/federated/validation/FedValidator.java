package org.lflang.federated.validation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;

/**
 * Helper class that is used to validate a federated reactor.
 */
public class FedValidator {

    public static void validateFederatedReactor(Reactor reactor, ErrorReporter errorReporter) {
        if (!reactor.isFederated()) return;

        // Construct the set of excluded reactions for this federate.
        // If a reaction is a network reaction that belongs to this federate, we
        // don't need to perform this analysis.
        Iterable<Reaction> reactions = ASTUtils.allReactions(reactor);
        for (Reaction react : reactions) {
            // Create a collection of all the VarRefs (i.e., triggers, sources, and effects) in the react
            // signature that are ports that reference federates.
            // We then later check that all these VarRefs reference this federate. If not, we will add this
            // react to the list of reactions that have to be excluded (note that mixing VarRefs from
            // different federates is not allowed).
            List<VarRef> allVarRefsReferencingFederates = new ArrayList<>();
            // Add all the triggers that are outputs
            Stream<VarRef> triggersAsVarRef = react.getTriggers().stream().filter(it -> it instanceof VarRef).map(it -> (VarRef) it);
            allVarRefsReferencingFederates.addAll(
                triggersAsVarRef.filter(it -> it.getVariable() instanceof Output).toList()
            );
            // Add all the sources that are outputs
            allVarRefsReferencingFederates.addAll(
                react.getSources().stream().filter(it -> it.getVariable() instanceof Output).toList()
            );
            // Add all the effects that are inputs
            allVarRefsReferencingFederates.addAll(
                react.getEffects().stream().filter(it -> it.getVariable() instanceof Input).toList()
            );
            containsAllVarRefs(allVarRefsReferencingFederates, errorReporter);
        }
    }

    /**
     * Check if this federate contains all the {@code varRefs}. If not, report an error using {@code errorReporter}.
     */
    private static void containsAllVarRefs(List<VarRef> varRefs, ErrorReporter errorReporter) {
        var referencesFederate = false;
        Instantiation instantiation = null;
        for (VarRef varRef : varRefs) {
            if (instantiation == null) {
                instantiation = varRef.getContainer();
                referencesFederate = true;
            } else if (!varRef.getContainer().equals(instantiation)) {
                    errorReporter.reportError(varRef, "Mixed triggers and effects from" +
                        " different federates. This is not permitted");
            }
        }

    }
}
