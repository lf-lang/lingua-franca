package org.lflang.generator.c;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.lflang.ASTUtils;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;

/**
 * Helper class to handle code generation of contained reactors.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
 */

public class InteractingContainedReactors {
    /**
     * Data structure that for each instantiation of a contained
     * reactor. This provides a set of input and output ports that trigger
     * reactions of the container, are read by a reaction of the
     * container, or that receive data from a reaction of the container.
     * For each port, this provides a list of reaction indices that
     * are triggered by the port, or an empty list if there are no
     * reactions triggered by the port.
     */
    // This horrible data structure is a collection, indexed by instantiation
    // of a contained reactor, of lists, indexed by ports of the contained reactor
    // that are referenced by reactions of the container, of reactions that are
    // triggered by the port of the contained reactor. The list is empty if
    // the port does not trigger reactions but is read by the reaction or
    // is written to by the reaction.
    LinkedHashMap<
        Instantiation, LinkedHashMap<
            Port, LinkedList<Integer>
        >
    > portsByContainedReactor = new LinkedHashMap<>();

    /**
     * Scan the reactions of the specified reactor and record which ports are
     * referenced by reactions and which reactions are triggered by such ports.
     */
    public InteractingContainedReactors(Reactor reactor) {
        var reactionCount = 0;
        for (Reaction reaction : ASTUtils.allReactions(reactor)) {
            // First, handle reactions that produce data sent to inputs
            // of contained reactors.
            for (VarRef effect : ASTUtils.convertToEmptyListIfNull(reaction.getEffects())) {
                // If an effect is an input, then it must be an input
                // of a contained reactor.
                if (effect.getVariable() instanceof Input) {
                    // This reaction is not triggered by the port, so
                    // we do not add it to the list returned by the following.
                    addPort(effect.getContainer(), (Input) effect.getVariable());
                }
            }
            // Second, handle reactions that are triggered by outputs
            // of contained reactors.
            for (TriggerRef trigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
                if (trigger instanceof VarRef triggerAsVarRef) {
                    // If an trigger is an output, then it must be an output
                    // of a contained reactor.
                    if (triggerAsVarRef.getVariable() instanceof Output) {
                        var list = addPort(triggerAsVarRef.getContainer(), (Output) triggerAsVarRef.getVariable());
                        list.add(reactionCount);
                    }
                }
            }
            // Third, handle reading (but not triggered by)
            // outputs of contained reactors.
            for (VarRef source : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
                if (source.getVariable() instanceof Output) {
                    // If an source is an output, then it must be an output
                    // of a contained reactor.
                    // This reaction is not triggered by the port, so
                    // we do not add it to the list returned by the following.
                    addPort(source.getContainer(), (Output) source.getVariable());
                }
            }
        // Increment the reaction count even if not in the federate for consistency.
        reactionCount++;
        }
    }

    /**
     * Return or create the list to which reactions triggered by the specified port
     * are to be added. This also records that the port is referenced by the
     * container's reactions.
     * @param containedReactor The contained reactor.
     * @param port The port.
     */
    private List<Integer> addPort(Instantiation containedReactor, Port port) {
        // Get or create the entry for the containedReactor.
        var containedReactorEntry = portsByContainedReactor.computeIfAbsent(
            containedReactor,
            k -> new LinkedHashMap<>()
        );
        // Get or create the entry for the port.
        return containedReactorEntry.computeIfAbsent(port, k -> new LinkedList<>());
    }

    /**
     * Return the set of contained reactors that have ports that are referenced
     * by reactions of the container reactor.
     */
    public Set<Instantiation> containedReactors() {
        return portsByContainedReactor.keySet();
    }

    /**
     * Return the set of ports of the specified contained reactor that are
     * referenced by reactions of the container reactor. Return an empty
     * set if there are none.
     * @param containedReactor The contained reactor.
     */
    public Set<Port> portsOfInstance(Instantiation containedReactor) {
        Set<Port> result = null;
        var ports = portsByContainedReactor.get(containedReactor);
        if (ports == null) {
            result = new LinkedHashSet<>();
        } else {
            result = ports.keySet();
        }
        return result;
    }

    /**
     * Return the indices of the reactions triggered by the specified port
     * of the specified contained reactor or an empty list if there are none.
     * @param containedReactor The contained reactor.
     * @param port The port.
     */
    public List<Integer> reactionsTriggered(Instantiation containedReactor, Port port) {
        var ports = portsByContainedReactor.get(containedReactor);
        if (ports != null) {
            var list = ports.get(port);
            if (list != null) {
                return list;
            }
        }
        return new LinkedList<>();
    }
}

