package org.lflang.generator.c;
import static org.lflang.generator.c.CMixedRadixGenerator.db;
import static org.lflang.generator.c.CMixedRadixGenerator.dc;
import static org.lflang.generator.c.CMixedRadixGenerator.dr;
import static org.lflang.generator.c.CMixedRadixGenerator.sb;
import static org.lflang.generator.c.CMixedRadixGenerator.sc;
import static org.lflang.generator.c.CMixedRadixGenerator.sr;
import static org.lflang.util.StringUtil.addDoubleQuotes;
import static org.lflang.util.StringUtil.joinObjects;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.AttributeUtils;
import org.lflang.TargetConfig;

import org.lflang.TargetProperty.LogLevel;
import org.lflang.federated.extensions.CExtensionUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;

import com.google.common.collect.Iterables;

/**
 * Generate code for the "_lf_initialize_trigger_objects" function
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
 */
public class CTriggerObjectsGenerator {
    /**
     * Generate the _lf_initialize_trigger_objects function for 'federate'.
     */
    public static String generateInitializeTriggerObjects(
        ReactorInstance main,
        TargetConfig targetConfig,
        CodeBuilder initializeTriggerObjects,
        CodeBuilder startTimeStep,
        CTypes types,
        String lfModuleName,
        int startTimeStepIsPresentCount
    ) {
        var code = new CodeBuilder();
        code.pr("void _lf_initialize_trigger_objects() {");
        code.indent();
        // Initialize the LF clock.
        code.pr(String.join("\n",
            "// Initialize the _lf_clock",
            "lf_initialize_clock();"
        ));

        // Initialize tracing if it is enabled
        if (targetConfig.tracing != null) {
            var traceFileName = lfModuleName;
            if (targetConfig.tracing.traceFileName != null) {
                traceFileName = targetConfig.tracing.traceFileName;
            }
            code.pr(String.join("\n",
                "// Initialize tracing",
                "start_trace("+ addDoubleQuotes(traceFileName + ".lft") + ");"
            )); // .lft is for Lingua Franca trace
        }

        // Create the table to initialize is_present fields to false between time steps.
        if (startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to _is_present fields.
            code.pr(String.join("\n",
                "// Create the array that will contain pointers to is_present fields to reset on each step.",
                "_lf_is_present_fields_size = "+startTimeStepIsPresentCount+";",
                "_lf_is_present_fields = (bool**)calloc("+startTimeStepIsPresentCount+", sizeof(bool*));",
                "if (_lf_is_present_fields == NULL) lf_print_error_and_exit(" + addDoubleQuotes("Out of memory!") + ");",
                "_lf_is_present_fields_abbreviated = (bool**)calloc("+startTimeStepIsPresentCount+", sizeof(bool*));",
                "if (_lf_is_present_fields_abbreviated == NULL) lf_print_error_and_exit(" + addDoubleQuotes("Out of memory!") + ");",
                "_lf_is_present_fields_abbreviated_size = 0;"
            ));
        }

        // Create the table to initialize intended tag fields to 0 between time
        // steps.
        if (startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to
            // intended_tag fields.
            // There is a 1-1 map between structs containing is_present and
            // intended_tag fields,
            // thus, we reuse startTimeStepIsPresentCount as the counter.
            code.pr(String.join("\n",
                                CExtensionUtils.surroundWithIfFederatedDecentralized("""
                                // Create the array that will contain pointers to intended_tag fields to reset on each step.
                                _lf_intended_tag_fields_size = %s;
                                _lf_intended_tag_fields = (tag_t**)malloc(_lf_intended_tag_fields_size * sizeof(tag_t*));
                                """.formatted(startTimeStepIsPresentCount)
                                )
            ));
        }


        code.pr(initializeTriggerObjects.toString());

        code.pr(deferredInitialize(
            main,
            main.reactions,
            targetConfig,
            types
        ));
        code.pr(deferredInitializeNonNested(
            main,
            main,
            main.reactions,
            types
        ));
        // Next, for every input port, populate its "self" struct
        // fields with pointers to the output port that sends it data.
        code.pr(deferredConnectInputsToOutputs(
            main
        ));
        // Put the code here to set up the tables that drive resetting is_present and
        // decrementing reference counts between time steps. This code has to appear
        // in _lf_initialize_trigger_objects() after the code that makes connections
        // between inputs and outputs.
        code.pr(startTimeStep.toString());
        code.pr(setReactionPriorities(
            main
        ));
        code.pr(generateSchedulerInitializer(
            main,
            targetConfig
        ));

        code.pr("""
        #ifdef EXECUTABLE_PREAMBLE
        _lf_executable_preamble();
        #endif
        """);

        // Initialize triggers for federated execution.
        code.pr(CExtensionUtils.surroundWithIfFederated("initialize_triggers_for_federate();"));

        code.unindent();
        code.pr("}\n");
        return code.toString();
    }

    /**
    * Generate code to initialize the scheduler for the threaded C runtime.
    */
    public static String generateSchedulerInitializer(
        ReactorInstance main,
        TargetConfig targetConfig
    ) {
        if (!targetConfig.threading) {
            return "";
        }
        var code = new CodeBuilder();
        var numReactionsPerLevel = main.assignLevels().getNumReactionsPerLevel();
        var numReactionsPerLevelJoined = Arrays.stream(numReactionsPerLevel)
                .map(String::valueOf)
                .collect(Collectors.joining(", "));
        code.pr(String.join("\n",
            "// Initialize the scheduler",
            "size_t num_reactions_per_level["+numReactionsPerLevel.length+"] = ",
            "    {" + numReactionsPerLevelJoined + "};",
            "sched_params_t sched_params = (sched_params_t) {",
            "                        .num_reactions_per_level = &num_reactions_per_level[0],",
            "                        .num_reactions_per_level_size = (size_t) "+numReactionsPerLevel.length+"};",
            "lf_sched_init(",
            "    (size_t)_lf_number_of_workers,",
            "    &sched_params",
            ");"
        ));
        return code.toString();
    }

    /**
     * * Set the reaction priorities based on dependency analysis.
     *
     * @param reactor The reactor on which to do this.
     */
    private static String setReactionPriorities(
        ReactorInstance reactor
    ) {
        var code = new CodeBuilder();
        setReactionPriorities(reactor, code);
        return code.toString();
    }

    /**
     * * Set the reaction priorities based on dependency analysis.
     *
     * @param reactor The reactor on which to do this.
     * @param builder Where to write the code.
     */
    private static boolean setReactionPriorities(
        ReactorInstance reactor,
        CodeBuilder builder
    ) {
        var foundOne = false;
        // Force calculation of levels if it has not been done.
        // FIXME: Comment out this as I think it is redundant.
        //  If it is NOT redundant then deadline propagation is not correct
        // reactor.assignLevels();

        // We handle four different scenarios
        //  1) A reactionInstance has 1 level and 1 deadline
        //  2) A reactionInstance has 1 level but multiple deadlines
        //  3) A reaction instance has multiple levels but all have the same deadline
        //  4) Multiple deadlines and levels

        var prolog = new CodeBuilder();
        var epilog = new CodeBuilder();

        for (ReactionInstance r : reactor.reactions) {
            var levelSet = r.getLevels();
            var deadlineSet = r.getInferredDeadlines();

            if (levelSet.size() > 1 || deadlineSet.size() > 1) {
                // Scenario 2-4
                if (prolog.length() == 0) {
                    prolog.startScopedBlock();
                    epilog.endScopedBlock();
                }
            }
            if (deadlineSet.size() > 1) {
                // Scenario (2) or (4)
                var deadlines = r.getInferredDeadlinesList().stream()
                                 .map(elt -> ("0x" + Long.toString(elt.toNanoSeconds(), 16) + "LL"))
                                 .collect(Collectors.toList());

                prolog.pr("interval_t "+r.uniqueID()+"_inferred_deadlines[] = { "+joinObjects(deadlines, ", ")+" };");
            }

            if (levelSet.size() > 1) {
                // Scenario (3) or (4)
                // Cannot use the above set of levels because it is a set, not a list.
                prolog.pr("int "+r.uniqueID()+"_levels[] = { "+joinObjects(r.getLevelsList(), ", ")+" };");
            }
        }


        var temp = new CodeBuilder();
        temp.pr("// Set reaction priorities for " + reactor);
        temp.startScopedBlock(reactor);
        for (ReactionInstance r : reactor.reactions) {
            //if (currentFederate.contains(r.getDefinition())) {
            foundOne = true;
            var levelSet = r.getLevels();
            var deadlineSet = r.getInferredDeadlines();

            // Get the head of the associated lists. To avoid duplication in
            //  several of the following cases
            var level = r.getLevelsList().get(0);
            var inferredDeadline = r.getInferredDeadlinesList().get(0);
            var runtimeIdx =CUtil.runtimeIndex(r.getParent());

            if (levelSet.size() == 1 && deadlineSet.size() == 1) {
                // Scenario (1)

                var indexValue = inferredDeadline.toNanoSeconds() << 16 | level;

                var reactionIndex = "0x" + Long.toUnsignedString(indexValue, 16) + "LL";

                temp.pr(String.join("\n",
                    CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                    "// index is the OR of level "+level+" and ",
                    "// deadline "+ inferredDeadline.toNanoSeconds()+" shifted left 16 bits.",
                    CUtil.reactionRef(r)+".index = "+reactionIndex+";"
                ));
            } else if (levelSet.size() == 1 && deadlineSet.size() > 1) {
                // Scenario 2
                temp.pr(String.join("\n",
                    CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                    "// index is the OR of levels["+runtimeIdx+"] and ",
                    "// deadlines["+runtimeIdx+"] shifted left 16 bits.",
                    CUtil.reactionRef(r)+".index = ("+r.uniqueID()+"_inferred_deadlines["+runtimeIdx+"] << 16) | " +
                        level+";"
                ));

            } else if (levelSet.size() > 1 && deadlineSet.size() == 1) {
                // Scenarion (3)
                temp.pr(String.join("\n",
                    CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                    "// index is the OR of levels["+runtimeIdx+"] and ",
                    "// deadlines["+runtimeIdx+"] shifted left 16 bits.",
                    CUtil.reactionRef(r)+".index = ("+inferredDeadline.toNanoSeconds()+" << 16) | " +
                        r.uniqueID()+"_levels["+runtimeIdx+"];"
                ));

            } else if (levelSet.size() > 1 && deadlineSet.size() > 1) {
                // Scenario (4)
                temp.pr(String.join("\n",
                    CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                    "// index is the OR of levels["+runtimeIdx+"] and ",
                    "// deadlines["+runtimeIdx+"] shifted left 16 bits.",
                    CUtil.reactionRef(r)+".index = ("+r.uniqueID()+"_inferred_deadlines["+runtimeIdx+"] << 16) | " +
                        r.uniqueID()+"_levels["+runtimeIdx+"];"
                ));
            }

        }
        for (ReactorInstance child : reactor.children) {
            foundOne = setReactionPriorities(child, temp) || foundOne;
        }
        temp.endScopedBlock();

        if (foundOne) {
            builder.pr(prolog.toString());
            builder.pr(temp.toString());
            builder.pr(epilog.toString());
        }
        return foundOne;
    }

    /**
     * Generate assignments of pointers in the "self" struct of a destination
     * port's reactor to the appropriate entries in the "self" struct of the
     * source reactor. This has to be done after all reactors have been created
     * because inputs point to outputs that are arbitrarily far away.
     * @param instance The reactor instance.
     */
    private static String deferredConnectInputsToOutputs(
        ReactorInstance instance
    ) {
        var code = new CodeBuilder();
        code.pr("// Connect inputs and outputs for reactor "+instance.getFullName()+".");
        // Iterate over all ports of this reactor that depend on reactions.
        for (PortInstance input : instance.inputs) {
            if (!input.getDependsOnReactions().isEmpty()) {
                // Input is written to by reactions in the parent of the port's parent.
                code.pr(connectPortToEventualDestinations(input));
            }
        }
        for (PortInstance output : instance.outputs) {
            if (!output.getDependsOnReactions().isEmpty()) {
                // Output is written to by reactions in the port's parent.
                code.pr(connectPortToEventualDestinations(output));
            }
        }
        for (ReactorInstance child: instance.children) {
            code.pr(deferredConnectInputsToOutputs(child));
        }
        return code.toString();
    }

    /**
     * Generate assignments of pointers in the "self" struct of a destination
     * port's reactor to the appropriate entries in the "self" struct of the
     * source reactor. If this port is an input, then it is being written
     * to by a reaction belonging to the parent of the port's parent.
     * If it is an output, then it is being written to by a reaction belonging
     * to the port's parent.
     * @param src A port that is written to by reactions.
     */
    private static String connectPortToEventualDestinations(
        PortInstance src
    ) {
        var code = new CodeBuilder();
        for (SendRange srcRange: src.eventualDestinations()) {
            for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                var dst = dstRange.instance;
                var destStructType = CGenerator.variableStructType(dst);

                // NOTE: For federated execution, dst.getParent() should always be contained
                // by the currentFederate because an AST transformation removes connections
                // between ports of distinct federates. So the following check is not
                // really necessary.
                var mod = (dst.isMultiport() || (src.isInput() && src.isMultiport()))? "" : "&";
                code.pr("// Connect "+srcRange+" to port "+dstRange);
                code.startScopedRangeBlock(srcRange, dstRange);
                if (src.isInput()) {
                    // Source port is written to by reaction in port's parent's parent
                    // and ultimate destination is further downstream.
                    code.pr(CUtil.portRef(dst, dr, db, dc)+" = ("+destStructType+"*)"+mod+CUtil.portRefNested(src, sr, sb, sc)+";");
                } else if (dst.isOutput()) {
                    // An output port of a contained reactor is triggering a reaction.
                    code.pr(CUtil.portRefNested(dst, dr, db, dc)+" = ("+destStructType+"*)&"+CUtil.portRef(src, sr, sb, sc)+";");
                } else {
                    // An output port is triggering an input port.
                    code.pr(CUtil.portRef(dst, dr, db, dc)+" = ("+destStructType+"*)&"+CUtil.portRef(src, sr, sb, sc)+";");
                    if (AttributeUtils.isSparse(dst.getDefinition())) {
                        code.pr(CUtil.portRef(dst, dr, db, dc)+"->sparse_record = "+CUtil.portRefName(dst, dr, db, dc)+"__sparse;");
                        code.pr(CUtil.portRef(dst, dr, db, dc)+"->destination_channel = "+dc+";");
                    }
                }
                code.endScopedRangeBlock(srcRange, dstRange);
            }
        }
        return code.toString();
    }

    /**
     * For each reaction of the specified reactor,
     * set the last_enabling_reaction field of the reaction struct to point
     * to the single dominating upstream reaction if there is one, or to be
     * NULL if there is none.
     *
     * @param r The reactor.
     */
    private static String deferredOptimizeForSingleDominatingReaction(
        ReactorInstance r
    ) {
        var code = new CodeBuilder();
        for (ReactionInstance reaction : r.reactions) {
                // The following code attempts to gather into a loop assignments of successive
                // bank members relations between reactions to avoid large chunks of inline code
                // when a large bank sends to a large bank or when a large bank receives from
                // one reaction that is either multicasting or sending through a multiport.
                var start = 0;
                var end = 0;
                var domStart = 0;
                var same = false; // Set to true when finding a string of identical dominating reactions.
                ReactionInstance.Runtime previousRuntime = null;
                var first = true;  //First time through the loop.
                for (ReactionInstance.Runtime runtime : reaction.getRuntimeInstances()) {
                    if (!first) { // Not the first time through the loop.
                        if (same) { // Previously seen at least two identical dominating.
                            if (runtime.dominating != previousRuntime.dominating) {
                                // End of streak of same dominating reaction runtime instance.
                                code.pr(printOptimizeForSingleDominatingReaction(
                                    previousRuntime, start, end, domStart, same
                                ));
                                same = false;
                                start = runtime.id;
                                domStart = (runtime.dominating != null) ? runtime.dominating.id : 0;
                            }
                        } else if (runtime.dominating == previousRuntime.dominating) {
                            // Start of a streak of identical dominating reaction runtime instances.
                            same = true;
                        } else if (runtime.dominating != null && previousRuntime.dominating != null
                            && runtime.dominating.getReaction() == previousRuntime.dominating.getReaction()
                        ) {
                            // Same dominating reaction even if not the same dominating runtime.
                            if (runtime.dominating.id != previousRuntime.dominating.id + 1) {
                                // End of a streak of contiguous runtimes.
                                printOptimizeForSingleDominatingReaction(
                                    previousRuntime, start, end, domStart, same
                                );
                                same = false;
                                start = runtime.id;
                                domStart = runtime.dominating.id;
                            }
                        } else {
                            // Different dominating reaction.
                            printOptimizeForSingleDominatingReaction(
                                previousRuntime, start, end, domStart, same
                            );
                            same = false;
                            start = runtime.id;
                            domStart = (runtime.dominating != null) ? runtime.dominating.id : 0;
                        }
                    }
                    first = false;
                    previousRuntime = runtime;
                    end++;
                }
                if (end > start) {
                    printOptimizeForSingleDominatingReaction(
                        previousRuntime, start, end, domStart, same
                    );
                }
        }
        return code.toString();
    }

    /**
     * Print statement that sets the last_enabling_reaction field of a reaction.
     */
    private static String printOptimizeForSingleDominatingReaction(
        ReactionInstance.Runtime runtime,
        int start,
        int end,
        int domStart,
        boolean same
    ) {
        var code = new CodeBuilder();
        var dominatingRef = "NULL";

        if (end > start + 1) {
            code.startScopedBlock();
            var reactionRef = CUtil.reactionRef(runtime.getReaction(), "i");
            if (runtime.dominating != null) {
                if (same) {
                    dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.getReaction(), "" + domStart) + ")";
                } else {
                    dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.getReaction(), "j++") + ")";
                }
            }
            code.pr(String.join("\n",
                "// "+runtime.getReaction().getFullName()+" dominating upstream reaction.",
                "int j = "+domStart+";",
                "for (int i = "+start+"; i < "+end+"; i++) {",
                "    "+reactionRef+".last_enabling_reaction = "+dominatingRef+";",
                "}"
            ));
           code.endScopedBlock();
        } else if (end == start + 1) {
            var reactionRef = CUtil.reactionRef(runtime.getReaction(), "" + start);
            if (runtime.dominating != null) {
                dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.getReaction(), "" + domStart) + ")";
            }
            code.pr(String.join("\n",
                "// "+runtime.getReaction().getFullName()+" dominating upstream reaction.",
                reactionRef+".last_enabling_reaction = "+dominatingRef+";"
            ));
        }
        return code.toString();
    }

    /**
     * For the specified reaction, for ports that it writes to,
     * fill the trigger table for triggering downstream reactions.
     *
     * @param reactions The reactions.
     */
    private static String deferredFillTriggerTable(
        Iterable<ReactionInstance> reactions
    ) {
        var code = new CodeBuilder();
        for (ReactionInstance reaction : reactions) {
            var name = reaction.getParent().getFullName();

            var reactorSelfStruct = CUtil.reactorRef(reaction.getParent(), sr);

            var foundPort = false;

            for (PortInstance port : Iterables.filter(reaction.effects, PortInstance.class)) {
                if (!foundPort) {
                    // Need a separate index for the triggers array for each bank member.
                    code.startScopedBlock();
                    code.pr("int triggers_index["+reaction.getParent().getTotalWidth()+"] = { 0 }; // Number of bank members with the reaction.");
                    foundPort = true;
                }
                // If the port is a multiport, then its channels may have different sets
                // of destinations. For ordinary ports, there will be only one range and
                // its width will be 1.
                // We generate the code to fill the triggers array first in a temporary code buffer,
                // so that we can simultaneously calculate the size of the total array.
                for (SendRange srcRange : port.eventualDestinations()) {
                    var srcNested = port.isInput();
                    code.startScopedRangeBlock(srcRange, sr, sb, sc, srcNested);

                    var triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"]++]";
                    // Skip ports whose parent is not in the federation.
                    // This can happen with reactions in the top-level that have
                    // as an effect a port in a bank.
                    code.pr(String.join("\n",
                        "// Reaction "+reaction.index+" of "+name+" triggers "+srcRange.destinations.size()+" downstream reactions",
                        "// through port "+port.getFullName()+".",
                        CUtil.reactionRef(reaction, sr)+".triggered_sizes[triggers_index["+sr+"]] = "+srcRange.destinations.size()+";",
                        "// For reaction "+reaction.index+" of "+name+", allocate an",
                        "// array of trigger pointers for downstream reactions through port "+port.getFullName(),
                        "trigger_t** trigger_array = (trigger_t**)_lf_allocate(",
                        "        "+srcRange.destinations.size()+", sizeof(trigger_t*),",
                        "        &"+reactorSelfStruct+"->base.allocations); ",
                        triggerArray+" = trigger_array;"
                    ));
                    code.endScopedRangeBlock(srcRange);
                }
            }
            var cumulativePortWidth = 0;
            for (PortInstance port : Iterables.filter(reaction.effects, PortInstance.class)) {
                // If this port does not have any destinations, do not generate code for it.
                if (port.eventualDestinations().isEmpty()) continue;

                code.pr("for (int i = 0; i < "+reaction.getParent().getTotalWidth()+"; i++) triggers_index[i] = "+cumulativePortWidth+";");
                for (SendRange srcRange : port.eventualDestinations()) {
                    var srcNested = srcRange.instance.isInput();
                    var multicastCount = 0;
                    for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                        var dst = dstRange.instance;

                        code.startScopedRangeBlock(srcRange, dstRange);

                        // If the source is nested, need to take into account the parent's bank index
                        // when indexing into the triggers array.
                        var triggerArray = "";
                        if (srcNested && port.getParent().getWidth() > 1) {
                            triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"] + "+sc+" + src_range_mr.digits[1] * src_range_mr.radixes[0]]";
                        } else {
                            triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"] + "+sc+"]";
                        }

                        if (dst.isOutput()) {
                            // Include this destination port only if it has at least one
                            // reaction in the federation.
                            var belongs = false;
                            for (ReactionInstance destinationReaction : dst.getDependentReactions()) {
                                belongs = true;
                            }
                            if (belongs) {
                                code.pr(String.join("\n",
                                    "// Port "+port.getFullName()+" has reactions in its parent's parent.",
                                    "// Point to the trigger struct for those reactions.",
                                    triggerArray+"["+multicastCount+"] = &"+CUtil.triggerRefNested(dst, dr, db)+";"
                                ));
                            } else {
                                // Put in a NULL pointer.
                                code.pr(String.join("\n",
                                    "// Port "+port.getFullName()+" has reactions in its parent's parent.",
                                    "// But those are not in the federation.",
                                    triggerArray+"["+multicastCount+"] = NULL;"
                                ));
                            }
                        } else {
                            // Destination is an input port.
                            code.pr(String.join("\n",
                                "// Point to destination port "+dst.getFullName()+"'s trigger struct.",
                                triggerArray+"["+multicastCount+"] = &"+CUtil.triggerRef(dst, dr)+";"
                            ));
                        }
                        code.endScopedRangeBlock(srcRange, dstRange);
                        multicastCount++;
                    }
                }
                // If the port is an input of a contained reactor, then we have to take
                // into account the bank width of the contained reactor.
                if (port.getParent() != reaction.getParent()) {
                    cumulativePortWidth += port.getWidth() * port.getParent().getWidth();
                } else {
                    cumulativePortWidth += port.getWidth();
                }
            }
            if (foundPort) code.endScopedBlock();
        }
        return code.toString();
    }

    /**
     * For each input port of a contained reactor that receives data
     * from one or more of the specified reactions, set the num_destinations
     * field of the corresponding port structs on the self struct of
     * the reaction's parent reactor equal to the total number of
     * destination reactors.
     * If the port has a token type, this also initializes it with a token.
     * @param reactions The reactions.
     * @param types The C types.
     */
    private static String deferredInputNumDestinations(
        Iterable<ReactionInstance> reactions,
        CTypes types
    ) {
        // We need the number of destination _reactors_, not the
        // number of destination ports nor the number of destination reactions.
        // One of the destination reactors may be the container of this
        // instance because it may have a reaction to an output of this instance.
        // Since a port may be written to by multiple reactions,
        // ensure that this is done only once.
        var portsHandled = new HashSet<PortInstance>();
        var code = new CodeBuilder();
        for (ReactionInstance reaction : reactions) {
            for (PortInstance port : Iterables.filter(reaction.effects, PortInstance.class)) {
                if (port.isInput() && !portsHandled.contains(port)) {
                    // Port is an input of a contained reactor that gets data from a reaction of this reactor.
                    portsHandled.add(port);
                    code.pr("// Set number of destination reactors for port "+port.getParent().getName()+"."+port.getName()+".");
                    // The input port may itself have multiple destinations.
                    for (SendRange sendingRange : port.eventualDestinations()) {
                        code.startScopedRangeBlock(sendingRange, sr, sb, sc, sendingRange.instance.isInput());
                        // Syntax is slightly different for a multiport output vs. single port.
                        var connector = (port.isMultiport())? "->" : ".";
                        code.pr(CUtil.portRefNested(port, sr, sb, sc)+connector+"num_destinations = "+sendingRange.getNumberOfDestinationReactors()+";");

                        // Initialize token types.
                        var type = ASTUtils.getInferredType(port.getDefinition());
                        if (CUtil.isTokenType(type, types)) {
                            // Create the template token that goes in the port struct.
                            var rootType = CUtil.rootType(types.getTargetType(type));
                            // If the rootType is 'void', we need to avoid generating the code
                            // 'sizeof(void)', which some compilers reject.
                            var size = (rootType.equals("void")) ? "0" : "sizeof("+rootType+")";
                            // If the port is a multiport, then the portRefNested is itself a pointer
                            // so we want its value, not its address.
                            var indirection = (port.isMultiport())? "" : "&";
                            code.startChannelIteration(port);
                            code.pr(String.join("\n",
                                    "_lf_initialize_template((token_template_t*)",
                                    "        "+indirection+"("+CUtil.portRefNested(port, sr, sb, sc)+"),",
                                             size+");"
                            ));
                            code.endChannelIteration(port);
                        }
                        
                        code.endScopedRangeBlock(sendingRange);
                    }
                }
            }
        }
        return code.toString();
    }

    /**
     * For each output port of the specified reactor,
     * set the num_destinations field of port structs on its self struct
     * equal to the total number of destination reactors. This is used
     * to initialize reference counts in dynamically allocated tokens
     * sent to other reactors.
     * @param reactor The reactor instance.
     */
    private static String deferredOutputNumDestinations(
        ReactorInstance reactor
    ) {
        // Reference counts are decremented by each destination reactor
        // at the conclusion of a time step. Hence, the initial reference
        // count should equal the number of destination _reactors_, not the
        // number of destination ports nor the number of destination reactions.
        // One of the destination reactors may be the container of this
        // instance because it may have a reaction to an output of this instance.
        var code = new CodeBuilder();
        for (PortInstance output : reactor.outputs) {
            for (SendRange sendingRange : output.eventualDestinations()) {
                code.pr("// For reference counting, set num_destinations for port " + output.getFullName() + ".");
                code.startScopedRangeBlock(sendingRange, sr, sb, sc, sendingRange.instance.isInput());
                code.pr(CUtil.portRef(output, sr, sb, sc)+".num_destinations = "+sendingRange.getNumberOfDestinationReactors()+";");
                code.endScopedRangeBlock(sendingRange);
            }
        }
        return code.toString();
    }

    /**
     * Perform initialization functions that must be performed after
     * all reactor runtime instances have been created.
     * This function does not create nested loops over nested banks,
     * so each function it calls must handle its own iteration
     * over all runtime instance.
     * @param reactor The container.
     * @param main The top-level reactor.
     * @param reactions The list of reactions to consider.
     * @param types The C types.
     */
    private static String deferredInitializeNonNested(
        ReactorInstance reactor,
        ReactorInstance main,
        Iterable<ReactionInstance> reactions,
        CTypes types
    ) {
        var code = new CodeBuilder();
        code.pr("// **** Start non-nested deferred initialize for "+reactor.getFullName());
        // Initialize the num_destinations fields of port structs on the self struct.
        // This needs to be outside the above scoped block because it performs
        // its own iteration over ranges.
        code.pr(deferredInputNumDestinations(
            reactions,
            types
        ));

        // Second batch of initializes cannot be within a for loop
        // iterating over bank members because they iterate over send
        // ranges which may span bank members.
        if (reactor != main) {
            code.pr(deferredOutputNumDestinations(
                reactor
            ));
        }
        code.pr(deferredFillTriggerTable(
            reactions
        ));
        code.pr(deferredOptimizeForSingleDominatingReaction(
            reactor
        ));
        for (ReactorInstance child: reactor.children) {
            code.pr(deferredInitializeNonNested(
                child,
                main,
                child.reactions,
                types
            ));
        }
        code.pr("// **** End of non-nested deferred initialize for "+reactor.getFullName());
        return code.toString();
    }

    /**
     * For each output of the specified reactor that has a token type
     * (type* or type[]), create a template token and put it on the self struct.
     * @param reactor The reactor.
     */
    private static String deferredCreateTemplateTokens(
        ReactorInstance reactor,
        CTypes types
    ) {
        var code = new CodeBuilder();
        // Look for outputs with token types.
        for (PortInstance output : reactor.outputs) {
            var type = ASTUtils.getInferredType(output.getDefinition());
            if (CUtil.isTokenType(type, types)) {
                // Create the template token that goes in the trigger struct.
                // Its reference count is zero, enabling it to be used immediately.
                var rootType = CUtil.rootType(types.getTargetType(type));
                // If the rootType is 'void', we need to avoid generating the code
                // 'sizeof(void)', which some compilers reject.
                var size = (rootType.equals("void")) ? "0" : "sizeof("+rootType+")";
                code.startChannelIteration(output);
                code.pr(String.join("\n",
                        "_lf_initialize_template((token_template_t*)",
                        "        &("+CUtil.portRef(output)+"),",
                                 size+");"
                ));
                code.endChannelIteration(output);
            }
        }
        return code.toString();
    }

    /**
     * For the specified reaction, for ports that it writes to,
     * set up the arrays that store the results (if necessary) and
     * that are used to trigger downstream reactions if an effect is actually
     * produced.  The port may be an output of the reaction's parent
     * or an input to a reactor contained by the parent.
     *
     * @param reaction The reaction instance.
     */
    private static String deferredReactionOutputs(
        ReactionInstance reaction,
        TargetConfig targetConfig
    ) {
        var code = new CodeBuilder();
        // val selfRef = CUtil.reactorRef(reaction.getParent());
        var name = reaction.getParent().getFullName();
        // Insert a string name to facilitate debugging.
        if (targetConfig.logLevel.compareTo(LogLevel.LOG) >= 0) {
            code.pr(CUtil.reactionRef(reaction)+".name = "+addDoubleQuotes(name+" reaction "+reaction.index)+";");
        }

        var reactorSelfStruct = CUtil.reactorRef(reaction.getParent());

        // Count the output ports and inputs of contained reactors that
        // may be set by this reaction. This ignores actions in the effects.
        // Collect initialization statements for the output_produced array for the reaction
        // to point to the is_present field of the appropriate output.
        // These statements must be inserted after the array is malloc'd,
        // but we construct them while we are counting outputs.
        var outputCount = 0;
        var init = new CodeBuilder();

        init.startScopedBlock();
        init.pr("int count = 0; SUPPRESS_UNUSED_WARNING(count);");
        for (PortInstance effect : Iterables.filter(reaction.effects, PortInstance.class)) {
            // Create the entry in the output_produced array for this port.
            // If the port is a multiport, then we need to create an entry for each
            // individual channel.

            // If the port is an input of a contained reactor, then, if that
            // contained reactor is a bank, we will have to iterate over bank
            // members.
            var bankWidth = 1;
            var portRef = "";
            if (effect.isInput()) {
                init.pr("// Reaction writes to an input of a contained reactor.");
                bankWidth = effect.getParent().getWidth();
                init.startScopedBlock(effect.getParent());
                portRef = CUtil.portRefNestedName(effect);
            } else {
                init.startScopedBlock();
                portRef = CUtil.portRefName(effect);
            }

            if (effect.isMultiport()) {
                // Form is slightly different for inputs vs. outputs.
                var connector = ".";
                if (effect.isInput()) connector = "->";

                // Point the output_produced field to where the is_present field of the port is.
                init.pr(String.join("\n",
                    "for (int i = 0; i < "+effect.getWidth()+"; i++) {",
                    "    "+CUtil.reactionRef(reaction)+".output_produced[i + count]",
                    "            = &"+portRef+"[i]"+connector+"is_present;",
                    "}",
                    "count += "+effect.getWidth()+";"
                ));
                outputCount += effect.getWidth() * bankWidth;
            } else {
                // The effect is not a multiport.
                init.pr(CUtil.reactionRef(reaction)+".output_produced[count++] = &"+portRef+".is_present;");
                outputCount += bankWidth;
            }
            init.endScopedBlock();
        }
        init.endScopedBlock();
        code.pr(String.join("\n",
            "// Total number of outputs (single ports and multiport channels)",
            "// produced by "+reaction+".",
            CUtil.reactionRef(reaction)+".num_outputs = "+outputCount+";"
        ));
        if (outputCount > 0) {
            code.pr(String.join("\n",
                "// Allocate memory for triggers[] and triggered_sizes[] on the reaction_t",
                "// struct for this reaction.",
                CUtil.reactionRef(reaction)+".triggers = (trigger_t***)_lf_allocate(",
                "        "+outputCount+", sizeof(trigger_t**),",
                "        &"+reactorSelfStruct+"->base.allocations);",
                CUtil.reactionRef(reaction)+".triggered_sizes = (int*)_lf_allocate(",
                "        "+outputCount+", sizeof(int),",
                "        &"+reactorSelfStruct+"->base.allocations);",
                CUtil.reactionRef(reaction)+".output_produced = (bool**)_lf_allocate(",
                "        "+outputCount+", sizeof(bool*),",
                "        &"+reactorSelfStruct+"->base.allocations);"
            ));
        }

        code.pr(String.join("\n",
            init.toString(),
            "// ** End initialization for reaction "+reaction.index+" of "+name
        ));
        return code.toString();
    }

    /**
     * Generate code to allocate the memory needed by reactions for triggering
     * downstream reactions.
     * @param reactions A list of reactions.
     */
    private static String deferredReactionMemory(
        Iterable<ReactionInstance> reactions,
        TargetConfig targetConfig
    ) {
        var code = new CodeBuilder();
        // For each reaction instance, allocate the arrays that will be used to
        // trigger downstream reactions.
        for (ReactionInstance reaction : reactions) {
            code.pr(deferredReactionOutputs(
                reaction,
                targetConfig
            ));
            var reactorSelfStruct = CUtil.reactorRef(reaction.getParent());

            // Next handle triggers of the reaction that come from a multiport output
            // of a contained reactor.  Also, handle startup and shutdown triggers.
            for (PortInstance trigger : Iterables.filter(reaction.triggers, PortInstance.class)) {
                // If the port is a multiport, then we need to create an entry for each
                // individual port.
                if (trigger.isMultiport() && trigger.getParent() != null && trigger.isOutput()) {
                    // Trigger is an output of a contained reactor or bank.
                    code.pr(String.join("\n",
                        "// Allocate memory to store pointers to the multiport output "+trigger.getName()+" ",
                        "// of a contained reactor "+trigger.getParent().getFullName()
                    ));
                    code.startScopedBlock(trigger.getParent());

                    var width = trigger.getWidth();
                    var portStructType = CGenerator.variableStructType(trigger);

                    code.pr(String.join("\n",
                        CUtil.reactorRefNested(trigger.getParent())+"."+trigger.getName()+"_width = "+width+";",
                        CUtil.reactorRefNested(trigger.getParent())+"."+trigger.getName(),
                        "        = ("+portStructType+"**)_lf_allocate(",
                        "                "+width+", sizeof("+portStructType+"*),",
                        "                &"+reactorSelfStruct+"->base.allocations); "
                    ));

                    code.endScopedBlock();
                }
            }
        }
        return code.toString();
    }

    /**
     * If any reaction of the specified reactor provides input
     * to a contained reactor, then generate code to allocate
     * memory to store the data produced by those reactions.
     * The allocated memory is pointed to by a field called
     * `_lf_containername.portname` on the self struct of the reactor.
     * @param reactor The reactor.
     */
    private static String deferredAllocationForEffectsOnInputs(
        ReactorInstance reactor
    ) {
        var code = new CodeBuilder();
        // Keep track of ports already handled. There may be more than one reaction
        // in the container writing to the port, but we want only one memory allocation.
        var portsHandled = new HashSet<PortInstance>();
        var reactorSelfStruct = CUtil.reactorRef(reactor);
        // Find parent reactions that mention multiport inputs of this reactor.
        for (ReactionInstance reaction : reactor.reactions) {
            for (PortInstance effect : Iterables.filter(reaction.effects, PortInstance.class)) {
                if (effect.getParent().getDepth() > reactor.getDepth() // port of a contained reactor.
                    && effect.isMultiport()
                    && !portsHandled.contains(effect)
                ) {
                    code.pr("// A reaction writes to a multiport of a child. Allocate memory.");
                    portsHandled.add(effect);
                    code.startScopedBlock(effect.getParent());
                    var portStructType = CGenerator.variableStructType(effect);
                    var effectRef = CUtil.portRefNestedName(effect);
                    code.pr(String.join("\n",
                        effectRef+"_width = "+effect.getWidth()+";",
                        "// Allocate memory to store output of reaction feeding ",
                        "// a multiport input of a contained reactor.",
                        effectRef+" = ("+portStructType+"**)_lf_allocate(",
                        "        "+effect.getWidth()+", sizeof("+portStructType+"*),",
                        "        &"+reactorSelfStruct+"->base.allocations); ",
                        "for (int i = 0; i < "+effect.getWidth()+"; i++) {",
                        "    "+effectRef+"[i] = ("+portStructType+"*)_lf_allocate(",
                        "            1, sizeof("+portStructType+"),",
                        "            &"+reactorSelfStruct+"->base.allocations); ",
                        "}"
                    ));
                    code.endScopedBlock();
                }
            }
        }
        return code.toString();
    }

    /**
     * Perform initialization functions that must be performed after
     * all reactor runtime instances have been created.
     * This function creates nested loops over nested banks.
     * @param reactor The container.
     */
    private static String deferredInitialize(
        ReactorInstance reactor,
        Iterable<ReactionInstance> reactions,
        TargetConfig targetConfig,
        CTypes types
    ) {
        var code = new CodeBuilder();
        code.pr("// **** Start deferred initialize for "+reactor.getFullName());
        // First batch of initializations is within a for loop iterating
        // over bank members for the reactor's parent.
        code.startScopedBlock(reactor);

        // If the child has a multiport that is an effect of some reaction in its container,
        // then we have to generate code to allocate memory for arrays pointing to
        // its data. If the child is a bank, then memory is allocated for the entire
        // bank width because a reaction cannot specify which bank members it writes
        // to so we have to assume it can write to any.
        code.pr(deferredAllocationForEffectsOnInputs(
            reactor
        ));
        code.pr(deferredReactionMemory(
            reactions,
            targetConfig
        ));

        // For outputs that are not primitive types (of form type* or type[]),
        // create a default token on the self struct.
        code.pr(deferredCreateTemplateTokens(
            reactor,
            types
        ));
        for (ReactorInstance child: reactor.children) {
            code.pr(deferredInitialize(
                child,
                child.reactions,
                targetConfig,
                types)
            );
        }
        code.endScopedBlock();
        code.pr("// **** End of deferred initialize for "+reactor.getFullName());
        return code.toString();
    }
}
