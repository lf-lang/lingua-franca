package org.lflang.generator.c;
import com.google.common.collect.Iterables;

import java.lang.annotation.Target;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Collectors;
import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TargetProperty.LogLevel;
import org.lflang.federated.CGeneratorExtension;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import static org.lflang.generator.c.CMixedRadixGenerator.*;
import static org.lflang.util.StringUtil.joinObjects;
import static org.lflang.util.StringUtil.addDoubleQuotes;

/**
 * Generate code for the "_lf_initialize_trigger_objects" function
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CTriggerObjectsGenerator {
    /**
     * Generate the _lf_initialize_trigger_objects function for 'federate'.
     */
    public static String generateInitializeTriggerObjects(
        FederateInstance federate,
        ReactorInstance main,
        TargetConfig targetConfig,
        CodeBuilder initializeTriggerObjects,
        CodeBuilder startTimeStep,
        CTypes types,
        String topLevelName,
        LinkedHashMap<String, Object> federationRTIProperties,
        int startTimeStepTokens,
        int startTimeStepIsPresentCount,
        int startupReactionCount,
        boolean isFederated,
        boolean isFederatedAndDecentralized,
        boolean clockSyncIsOn
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
            var traceFileName = topLevelName;
            if (targetConfig.tracing.traceFileName != null) {
                traceFileName = targetConfig.tracing.traceFileName;
                // Since all federates would have the same name, we need to append the federate name.
                if (isFederated) {
                    traceFileName += "_" + federate.name;
                }
            }
            code.pr(String.join("\n", 
                "// Initialize tracing",
                "start_trace("+ addDoubleQuotes(traceFileName + ".lft") + ");"
            )); // .lft is for Lingua Franca trace
        }

        // Create the table used to decrement reference counts between time steps.
        if (startTimeStepTokens > 0) {
            // Allocate the initial (before mutations) array of pointers to tokens.
            code.pr(String.join("\n", 
                "_lf_tokens_with_ref_count_size = "+startTimeStepTokens+";",
                "_lf_tokens_with_ref_count = (token_present_t*)calloc("+startTimeStepTokens+", sizeof(token_present_t));",
                "if (_lf_tokens_with_ref_count == NULL) error_print_and_exit(" + addDoubleQuotes("Out of memory!") + ");"
            ));
        }
        // Create the table to initialize is_present fields to false between time steps.
        if (startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to _is_present fields.
            code.pr(String.join("\n", 
                "// Create the array that will contain pointers to is_present fields to reset on each step.",
                "_lf_is_present_fields_size = "+startTimeStepIsPresentCount+";",
                "_lf_is_present_fields = (bool**)calloc("+startTimeStepIsPresentCount+", sizeof(bool*));",
                "if (_lf_is_present_fields == NULL) error_print_and_exit(" + addDoubleQuotes("Out of memory!") + ");",
                "_lf_is_present_fields_abbreviated = (bool**)calloc("+startTimeStepIsPresentCount+", sizeof(bool*));",
                "if (_lf_is_present_fields_abbreviated == NULL) error_print_and_exit(" + addDoubleQuotes("Out of memory!") + ");",
                "_lf_is_present_fields_abbreviated_size = 0;"
            ));
        }

        // Allocate the memory for triggers used in federated execution
        code.pr(CGeneratorExtension.allocateTriggersForFederate(federate, startTimeStepIsPresentCount, isFederated, isFederatedAndDecentralized));
        code.pr(String.join("\n",
            "_lf_startup_reactions = (reaction_t**)calloc(" + startupReactionCount + ", sizeof(reaction_t*));",
            "_lf_startup_reactions_size = " + startupReactionCount + ";"
        ));

        code.pr(initializeTriggerObjects.toString());
        // Assign appropriate pointers to the triggers
        // FIXME: For python target, almost surely in the wrong place.
        code.pr(CGeneratorExtension.initializeTriggerForControlReactions(main, main, federate));

        var reactionsInFederate = Iterables.filter(
            main.reactions, 
            r -> federate.contains(r.getDefinition())
        );
        
        code.pr(deferredInitialize(
            federate, 
            main, 
            reactionsInFederate,
            targetConfig,
            types,
            isFederated
        ));
        code.pr(deferredInitializeNonNested(
            federate,
            main,
            main,
            reactionsInFederate,
            isFederated
        ));
        // Next, for every input port, populate its "self" struct
        // fields with pointers to the output port that sends it data.
        code.pr(deferredConnectInputsToOutputs(
            federate,
            main,
            isFederated
        ));
        // Put the code here to set up the tables that drive resetting is_present and
        // decrementing reference counts between time steps. This code has to appear
        // in _lf_initialize_trigger_objects() after the code that makes connections
        // between inputs and outputs.
        code.pr(startTimeStep.toString());
        code.pr(setReactionPriorities(
            federate,
            main,
            isFederated
        ));
        code.pr(initializeFederate(
            federate, main, targetConfig, 
            federationRTIProperties,
            isFederated, 
            clockSyncIsOn
        ));
        code.pr(generateSchedulerInitializer(
            main,
            targetConfig
        ));
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
     * If the number of federates is greater than one, then generate the code
     * that initializes global variables that describe the federate.
     * @param federate The federate instance.
     */
    private static String initializeFederate(
        FederateInstance federate,
        ReactorInstance main,
        TargetConfig targetConfig,
        Map<String, Object> federationRTIProperties,
        boolean isFederated,
        boolean clockSyncIsOn
    ) {
        var code = new CodeBuilder();
        if (!isFederated) {
            return "";
        }
        code.pr("// ***** Start initializing the federated execution. */");        
        code.pr(String.join("\n", 
            "// Initialize the socket mutex",
            "lf_mutex_init(&outbound_socket_mutex);",
            "lf_cond_init(&port_status_changed);"
        ));
        
        if (isFederated && targetConfig.coordination == CoordinationType.DECENTRALIZED) {
            var reactorInstance = main.getChildReactorInstance(federate.instantiation);
            for (ParameterInstance param : reactorInstance.parameters) {
                if (param.getName().equalsIgnoreCase("STP_offset") && param.type.isTime) {
                    var stp = ASTUtils.getLiteralTimeValue(param.getInitialValue().get(0));
                    if (stp != null) {                        
                        code.pr("set_stp_offset("+GeneratorBase.timeInTargetLanguage(stp)+");");
                    }
                }
            }
        }
        
        // Set indicator variables that specify whether the federate has
        // upstream logical connections.
        if (federate.dependsOn.size() > 0) {
            code.pr("_fed.has_upstream  = true;");
        }
        if (federate.sendsTo.size() > 0) {
            code.pr("_fed.has_downstream = true;");
        }
        // Set global variable identifying the federate.
        code.pr("_lf_my_fed_id = "+federate.id+";");
        
        // We keep separate record for incoming and outgoing p2p connections to allow incoming traffic to be processed in a separate
        // thread without requiring a mutex lock.
        var numberOfInboundConnections = federate.inboundP2PConnections.size();
        var numberOfOutboundConnections  = federate.outboundP2PConnections.size();
        
        code.pr(String.join("\n", 
            "_fed.number_of_inbound_p2p_connections = "+numberOfInboundConnections+";",
            "_fed.number_of_outbound_p2p_connections = "+numberOfOutboundConnections+";"
        ));
        if (numberOfInboundConnections > 0) {
            code.pr(String.join("\n", 
                "// Initialize the array of socket for incoming connections to -1.",
                "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
                "    _fed.sockets_for_inbound_p2p_connections[i] = -1;",
                "}"
            ));
        }
        if (numberOfOutboundConnections > 0) {                        
            code.pr(String.join("\n", 
                "// Initialize the array of socket for outgoing connections to -1.",
                "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
                "    _fed.sockets_for_outbound_p2p_connections[i] = -1;",
                "}"
            ));
        }

        // If a test clock offset has been specified, insert code to set it here.
        if (targetConfig.clockSyncOptions.testOffset != null) {
            code.pr("set_physical_clock_offset((1 + "+federate.id+") * "+targetConfig.clockSyncOptions.testOffset.toNanoSeconds()+"LL);");
        }
        
        code.pr(String.join("\n", 
            "// Connect to the RTI. This sets _fed.socket_TCP_RTI and _lf_rti_socket_UDP.",
            "connect_to_rti("+addDoubleQuotes(federationRTIProperties.get("host").toString())+", "+federationRTIProperties.get("port")+");"
        ));
        
        // Disable clock synchronization for the federate if it resides on the same host as the RTI,
        // unless that is overridden with the clock-sync-options target property.
        if (clockSyncIsOn) {
            code.pr("synchronize_initial_physical_clock_with_rti(_fed.socket_TCP_RTI);");
        }
    
        if (numberOfInboundConnections > 0) {
            code.pr(String.join("\n", 
                "// Create a socket server to listen to other federates.",
                "// If a port is specified by the user, that will be used",
                "// as the only possibility for the server. If not, the port",
                "// will start from STARTING_PORT. The function will",
                "// keep incrementing the port until the number of tries reaches PORT_RANGE_LIMIT.",
                "create_server("+federate.port+");",
                "// Connect to remote federates for each physical connection.",
                "// This is done in a separate thread because this thread will call",
                "// connect_to_federate for each outbound physical connection at the same",
                "// time that the new thread is listening for such connections for inbound",
                "// physical connections. The thread will live until all connections",
                "// have been established.",
                "lf_thread_create(&_fed.inbound_p2p_handling_thread_id, handle_p2p_connections_from_federates, NULL);"
            ));
        }

        for (FederateInstance remoteFederate : federate.outboundP2PConnections) {
            code.pr("connect_to_federate("+remoteFederate.id+");");
        }
        return code.toString();
    }

    /**
     * * Set the reaction priorities based on dependency analysis.
     * 
     * @param currentFederate The federate to generate code for.
     * @param reactor The reactor on which to do this.
     * @param isFederated True if program is federated, false otherwise.
     */
    private static String setReactionPriorities(
        FederateInstance currentFederate,
        ReactorInstance reactor,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        setReactionPriorities(currentFederate, reactor, code, isFederated);
        return code.toString();
    }

    /**
     * * Set the reaction priorities based on dependency analysis.
     * 
     * @param currentFederate The federate to generate code for.
     * @param reactor The reactor on which to do this.
     * @param builder Where to write the code.
     * @param isFederated True if program is federated, false otherwise.
     */
    private static boolean setReactionPriorities(
        FederateInstance currentFederate,
        ReactorInstance reactor, 
        CodeBuilder builder,
        boolean isFederated
    ) {
        var foundOne = false;
        // Force calculation of levels if it has not been done.
        reactor.assignLevels();
        
        // If any reaction has multiple levels, then we need to create
        // an array with the levels here, before entering the iteration over banks.
        var prolog = new CodeBuilder();
        var epilog = new CodeBuilder();
        for (ReactionInstance r : reactor.reactions) {
            if (currentFederate.contains(r.getDefinition())) {
                var levels = r.getLevels();
                if (levels.size() != 1) {
                    if (prolog.length() == 0) {
                        prolog.startScopedBlock();
                        epilog.endScopedBlock();
                    }
                    // Cannot use the above set of levels because it is a set, not a list.
                    prolog.pr("int "+r.uniqueID()+"_levels[] = { "+joinObjects(r.getLevelsList(), ", ")+" };");
                }
            }
        }

        var temp = new CodeBuilder();
        temp.pr("// Set reaction priorities for " + reactor.toString());
        temp.startScopedBlock(reactor, currentFederate, isFederated, true);
        for (ReactionInstance r : reactor.reactions) {
            if (currentFederate.contains(r.getDefinition())) {
                foundOne = true;
                // The most common case is that all runtime instances of the
                // reaction have the same level, so deal with that case
                // specially.
                var levels = r.getLevels();
                if (levels.size() == 1) {
                    var level = -1;
                    for (Integer l : levels) {
                        level = l;
                    }
                    // xtend doesn't support bitwise operators...
                    var indexValue = r.deadline.toNanoSeconds() << 16 | level;
                    var reactionIndex = "0x" + Long.toString(indexValue, 16) + "LL";

                    temp.pr(String.join("\n", 
                        CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                        "// index is the OR of level "+level+" and ",
                        "// deadline "+r.deadline.toNanoSeconds()+" shifted left 16 bits.",
                        CUtil.reactionRef(r)+".index = "+reactionIndex+";"
                    ));
                } else {
                    var reactionDeadline = "0x" + Long.toString(r.deadline.toNanoSeconds(), 16) + "LL";

                    temp.pr(String.join("\n", 
                        CUtil.reactionRef(r)+".chain_id = "+r.chainID+";",
                        "// index is the OR of levels["+CUtil.runtimeIndex(r.getParent())+"] and ",
                        "// deadline "+r.deadline.toNanoSeconds()+" shifted left 16 bits.",
                        CUtil.reactionRef(r)+".index = ("+reactionDeadline+" << 16) | "+r.uniqueID()+"_levels["+CUtil.runtimeIndex(r.getParent())+"];"
                    ));
                }
            }
        }
        for (ReactorInstance child : reactor.children) {
            if (currentFederate.contains(child)) {
                foundOne = setReactionPriorities(currentFederate, child, temp, isFederated) || foundOne;
            }
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
        FederateInstance currentFederate,
        ReactorInstance instance,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        code.pr("// Connect inputs and outputs for reactor "+instance.getFullName()+".");
        // Iterate over all ports of this reactor that depend on reactions.
        for (PortInstance input : instance.inputs) {
            if (!input.getDependsOnReactions().isEmpty()) {
                // Input is written to by reactions in the parent of the port's parent.
                code.pr(connectPortToEventualDestinations(currentFederate, input, isFederated)); 
            }
        }
        for (PortInstance output : instance.outputs) {
            if (!output.getDependsOnReactions().isEmpty()) {
                // Output is written to by reactions in the port's parent.
                code.pr(connectPortToEventualDestinations(currentFederate, output, isFederated)); 
            }
        }
        for (ReactorInstance child: instance.children) {
            code.pr(deferredConnectInputsToOutputs(currentFederate, child, isFederated));
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
     * @param port A port that is written to by reactions.
     */
    private static String connectPortToEventualDestinations(
        FederateInstance currentFederate,
        PortInstance src,
        boolean isFederated
    ) {
        if (!currentFederate.contains(src.getParent())) return "";
        var code = new CodeBuilder();
        for (SendRange srcRange: src.eventualDestinations()) {
            for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                var dst = dstRange.instance;
                var destStructType = CGenerator.variableStructType(dst);
                
                // NOTE: For federated execution, dst.getParent() should always be contained
                // by the currentFederate because an AST transformation removes connections
                // between ports of distinct federates. So the following check is not
                // really necessary.
                if (currentFederate.contains(dst.getParent())) {
                    var mod = (dst.isMultiport() || (src.isInput() && src.isMultiport()))? "" : "&";
                    code.pr("// Connect "+srcRange.toString()+" to port "+dstRange.toString());
                    code.startScopedRangeBlock(currentFederate, srcRange, dstRange, isFederated);
                    if (src.isInput()) {
                        // Source port is written to by reaction in port's parent's parent
                        // and ultimate destination is further downstream.
                        code.pr(CUtil.portRef(dst, dr, db, dc)+" = ("+destStructType+"*)"+mod+CUtil.portRefNested(src, sr, sb, sc)+";");
                    } else if (dst.isOutput()) {
                        // An output port of a contained reactor is triggering a reaction.
                        code.pr(CUtil.portRefNested(dst, dr, db, dc)+" = ("+destStructType+"*)&"+CUtil.portRef(src, sr, sb, sc)+";");
                    } else {
                        // An output port is triggering
                        code.pr(CUtil.portRef(dst, dr, db, dc)+" = ("+destStructType+"*)&"+CUtil.portRef(src, sr, sb, sc)+";");
                    }
                    code.endScopedRangeBlock(srcRange, dstRange, isFederated);
                }
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
     * @param reactor The reactor.
     */
    private static String deferredOptimizeForSingleDominatingReaction(
        FederateInstance currentFederate,
        ReactorInstance r,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        for (ReactionInstance reaction : r.reactions) {
            if (currentFederate.contains(reaction.getDefinition())
                && currentFederate.contains(reaction.getParent())
            ) {
                
                // For federated systems, the above test may not be enough if there is a bank
                // of federates.  Calculate the divisor needed to compute the federate bank
                // index from the instance index of the reaction.
                var divisor = 1;
                if (isFederated) {
                    var parent = reaction.getParent();
                    while (parent.getDepth() > 1) {
                        divisor *= parent.getWidth();
                        parent = parent.getParent();
                    }
                }
                
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
                                    currentFederate, previousRuntime, start, end, domStart, same, divisor, isFederated
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
                                    currentFederate, previousRuntime, start, end, domStart, same, divisor, isFederated
                                );
                                same = false;
                                start = runtime.id;
                                domStart = runtime.dominating.id;
                            }
                        } else {
                            // Different dominating reaction.
                            printOptimizeForSingleDominatingReaction(
                                currentFederate, previousRuntime, start, end, domStart, same, divisor, isFederated
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
                        currentFederate, previousRuntime, start, end, domStart, same, divisor, isFederated
                    );
                }
            }
        }
        return code.toString();
    }

    /**
     * Print statement that sets the last_enabling_reaction field of a reaction.
     */
    private static String printOptimizeForSingleDominatingReaction(
        FederateInstance currentFederate,
        ReactionInstance.Runtime runtime, 
        int start, 
        int end, 
        int domStart, 
        boolean same, 
        int divisor,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        var domDivisor = 1;
        if (isFederated && runtime.dominating != null) {
            var domReaction = runtime.dominating.getReaction();
            // No need to do anything if the dominating reaction is not in the federate.
            // Note that this test is imperfect because the current federate may be a
            // bank member.
            if (!currentFederate.contains(domReaction.getDefinition())
                    || !currentFederate.contains(domReaction.getParent())) {
                return "";
            }
            // To really know whether the dominating reaction is in the federate,
            // we need to calculate a divisor for its runtime index. 
            var parent = runtime.dominating.getReaction().getParent();
            while (parent.getDepth() > 1) {
                domDivisor *= parent.getWidth();
                parent = parent.getParent();
            }
        }
        
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
                (isFederated ? 
                "    if (i / "+divisor+" != "+currentFederate.bankIndex+") continue; // Reaction is not in the federate." : 
                ""),
                (runtime.dominating != null ? 
                "    if (j / "+domDivisor+" != "+currentFederate.bankIndex+") continue; // Dominating reaction is not in the federate." : 
                ""), 
                "    "+reactionRef+".last_enabling_reaction = "+dominatingRef+";",
                "}"
            ));
           code.endScopedBlock();
        } else if (end == start + 1) {
            var reactionRef = CUtil.reactionRef(runtime.getReaction(), "" + start);
            if (runtime.dominating != null
                && (domDivisor == 1 || domStart/domDivisor == currentFederate.bankIndex)
            ) {
                dominatingRef =  "&(" + CUtil.reactionRef(runtime.dominating.getReaction(), "" + domStart) + ")";
            }
            if (!isFederated 
                || (start/divisor == currentFederate.bankIndex) 
                && (runtime.dominating == null || domStart/domDivisor == currentFederate.bankIndex)
            ) {
                code.pr(String.join("\n", 
                    "// "+runtime.getReaction().getFullName()+" dominating upstream reaction.",
                    reactionRef+".last_enabling_reaction = "+dominatingRef+";"
                ));
            }
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
        FederateInstance currentFederate,
        Iterable<ReactionInstance> reactions,
        boolean isFederated
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
                    var srcNested = (port.isInput())? true : false;
                    code.startScopedRangeBlock(currentFederate, srcRange, sr, sb, sc, srcNested, isFederated, true);
                    
                    var triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"]++]";
                    // Skip ports whose parent is not in the federation.
                    // This can happen with reactions in the top-level that have
                    // as an effect a port in a bank.
                    if (currentFederate.contains(port.getParent())) {
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
                    } else {
                        // Port is not in the federate or has no destinations.
                        // Set the triggered_width fields to 0.
                        code.pr(CUtil.reactionRef(reaction, sr)+".triggered_sizes["+sc+"] = 0;");
                    }
                    code.endScopedRangeBlock(srcRange, isFederated);
                }
            }
            var cumulativePortWidth = 0;
            for (PortInstance port : Iterables.filter(reaction.effects, PortInstance.class)) {
                code.pr("for (int i = 0; i < "+reaction.getParent().getTotalWidth()+"; i++) triggers_index[i] = "+cumulativePortWidth+";");
                for (SendRange srcRange : port.eventualDestinations()) {
                    if (currentFederate.contains(port.getParent())) {
                        var srcNested = srcRange.instance.isInput();
                        var multicastCount = 0;
                        for (RuntimeRange<PortInstance> dstRange : srcRange.destinations) {
                            var dst = dstRange.instance;
                                                        
                            code.startScopedRangeBlock(currentFederate, srcRange, dstRange, isFederated);
                            
                            // If the source is nested, need to take into account the parent's bank index
                            // when indexing into the triggers array.
                            var triggerArray = "";
                            if (srcNested && port.getParent().getWidth() > 1 && !(isFederated && port.getParent().getDepth() == 1)) {
                                triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"] + "+sc+" + src_range_mr.digits[1] * src_range_mr.radixes[0]]";
                            } else {
                                triggerArray = CUtil.reactionRef(reaction, sr)+".triggers[triggers_index["+sr+"] + "+sc+"]";
                            }
                                                                                        
                            if (dst.isOutput()) {
                                // Include this destination port only if it has at least one
                                // reaction in the federation.
                                var belongs = false;
                                for (ReactionInstance destinationReaction : dst.getDependentReactions()) {
                                    if (currentFederate.contains(destinationReaction.getParent())) {
                                        belongs = true;
                                    }
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
                            code.endScopedRangeBlock(srcRange, dstRange, isFederated);
                            multicastCount++;
                        }
                    }
                }
                cumulativePortWidth += port.getWidth();
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
     * destination reactors. This is used to initialize reference
     * counts in dynamically allocated tokens sent to other reactors.
     * @param reactions The reactions.
     */
    private static String deferredInputNumDestinations(
        FederateInstance currentFederate,
        Iterable<ReactionInstance> reactions,
        boolean isFederated
    ) {
        // Reference counts are decremented by each destination reactor
        // at the conclusion of a time step. Hence, the initial reference
        // count should equal the number of destination _reactors_, not the
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
                    code.pr("// For reference counting, set num_destinations for port "+port.getParent().getName()+"."+port.getName()+".");
                    // The input port may itself have multiple destinations.
                    for (SendRange sendingRange : port.eventualDestinations()) {
                        code.startScopedRangeBlock(currentFederate, sendingRange, sr, sb, sc, sendingRange.instance.isInput(), isFederated, true);
                        // Syntax is slightly different for a multiport output vs. single port.
                        var connector = (port.isMultiport())? "->" : ".";
                        code.pr(CUtil.portRefNested(port, sr, sb, sc)+connector+"num_destinations = "+sendingRange.getNumberOfDestinationReactors()+";");
                        code.endScopedRangeBlock(sendingRange, isFederated);
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
        FederateInstance currentFederate,
        ReactorInstance reactor,
        boolean isFederated
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
                code.startScopedRangeBlock(currentFederate, sendingRange, sr, sb, sc, sendingRange.instance.isInput(), isFederated, true);
                code.pr(CUtil.portRef(output, sr, sb, sc)+".num_destinations = "+sendingRange.getNumberOfDestinationReactors()+";");
                code.endScopedRangeBlock(sendingRange, isFederated);
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
     * @param federate The federate (used to determine whether a
     *  reaction belongs to the federate).
     */
    private static String deferredInitializeNonNested(
        FederateInstance currentFederate,
        ReactorInstance reactor, 
        ReactorInstance main,
        Iterable<ReactionInstance> reactions,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        code.pr("// **** Start non-nested deferred initialize for "+reactor.getFullName());    
        // Initialize the num_destinations fields of port structs on the self struct.
        // This needs to be outside the above scoped block because it performs
        // its own iteration over ranges.
        code.pr(deferredInputNumDestinations(
            currentFederate,
            reactions,
            isFederated
        ));
        
        // Second batch of initializes cannot be within a for loop
        // iterating over bank members because they iterate over send
        // ranges which may span bank members.
        if (reactor != main) {
            code.pr(deferredOutputNumDestinations(
                currentFederate,
                reactor,
                isFederated
            ));
        }
        code.pr(deferredFillTriggerTable(
            currentFederate,
            reactions,
            isFederated
        ));
        code.pr(deferredOptimizeForSingleDominatingReaction(
            currentFederate,
            reactor,
            isFederated
        ));
        for (ReactorInstance child: reactor.children) {
            if (currentFederate.contains(child)) {
                code.pr(deferredInitializeNonNested(
                    currentFederate, 
                    child, 
                    main,
                    child.reactions, 
                    isFederated
                ));
            }
        }
        code.pr("// **** End of non-nested deferred initialize for "+reactor.getFullName());
        return code.toString();
    }

    /** 
     * For each output of the specified reactor that has a token type
     * (type* or type[]), create a default token and put it on the self struct.
     * @param parent The reactor.
     */
    private static String deferredCreateDefaultTokens(
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
                code.pr(CUtil.portRef(output)+".token = _lf_create_token("+size+");");
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
     * @param The reaction instance.
     */
    private static String deferredReactionOutputs(
        FederateInstance currentFederate,
        ReactionInstance reaction,
        TargetConfig targetConfig,
        boolean isFederated
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
        init.pr("int count = 0;");
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
                init.startScopedBlock(effect.getParent(), currentFederate, isFederated, true);
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
            "// produced by "+reaction.toString()+".",
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
        FederateInstance currentFederate,
        Iterable<ReactionInstance> reactions,
        TargetConfig targetConfig,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        // For each reaction instance, allocate the arrays that will be used to
        // trigger downstream reactions.
        for (ReactionInstance reaction : reactions) {
            code.pr(deferredReactionOutputs(
                currentFederate,
                reaction,
                targetConfig,
                isFederated
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
                    code.startScopedBlock(trigger.getParent(), currentFederate, isFederated, true);
                    
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
        FederateInstance currentFederate,
        ReactorInstance reactor,
        boolean isFederated
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
                    && currentFederate.contains(effect.getParent())
                ) {
                    code.pr("// A reaction writes to a multiport of a child. Allocate memory.");
                    portsHandled.add(effect);
                    code.startScopedBlock(effect.getParent(), currentFederate, isFederated, true);
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
     * @param federate The federate (used to determine whether a
     *  reaction belongs to the federate).
     */
    private static String deferredInitialize(
        FederateInstance currentFederate,
        ReactorInstance reactor, 
        Iterable<ReactionInstance> reactions,
        TargetConfig targetConfig,
        CTypes types,
        boolean isFederated
    ) {
        if (!currentFederate.contains(reactor)) {
            return "";
        }
        var code = new CodeBuilder();
        code.pr("// **** Start deferred initialize for "+reactor.getFullName());
        // First batch of initializations is within a for loop iterating
        // over bank members for the reactor's parent.
        code.startScopedBlock(reactor, currentFederate, isFederated, true);
        
        // If the child has a multiport that is an effect of some reaction in its container,
        // then we have to generate code to allocate memory for arrays pointing to
        // its data. If the child is a bank, then memory is allocated for the entire
        // bank width because a reaction cannot specify which bank members it writes
        // to so we have to assume it can write to any.
        code.pr(deferredAllocationForEffectsOnInputs(
            currentFederate,
            reactor,
            isFederated
        ));
        code.pr(deferredReactionMemory(
            currentFederate,
            reactions,
            targetConfig,
            isFederated
        ));

        // For outputs that are not primitive types (of form type* or type[]),
        // create a default token on the self struct.
        code.pr(deferredCreateDefaultTokens(
            reactor,
            types
        ));
        for (ReactorInstance child: reactor.children) {
            if (currentFederate.contains(child)) {
                code.pr(deferredInitialize(
                    currentFederate, 
                    child, 
                    child.reactions, 
                    targetConfig, 
                    types, 
                    isFederated)
                );
            }
        }
        code.endScopedBlock();
        code.pr("// **** End of deferred initialize for "+reactor.getFullName());
        return code.toString();
    }
}
