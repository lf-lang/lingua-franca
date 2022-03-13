package org.lflang.generator.c;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

import javax.sound.sampled.Port;

import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.Input;
import static org.lflang.generator.c.CMixedRadixGenerator.*;
import static org.lflang.util.StringUtil.joinObjects;

/**
 * Generate code for the "_lf_initialize_trigger_objects" function
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CTriggerObjectsGenerator {
    public static String generateInitializeTriggerObjects() {
        return "";
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
                .collect(Collectors.joining(", \n"));
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
    public static String initializeFederate(
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
            "connect_to_rti(\""+federationRTIProperties.get("host")+"\", "+federationRTIProperties.get("port")+");"
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

    public static String setReactionPriorities(
        FederateInstance currentFederate,
        ReactorInstance reactor,
        boolean isFederated
    ) {
        var code = new CodeBuilder();
        setReactionPriorities(currentFederate, reactor, code, isFederated);
        return code.toString();
    }

    /** 
     * Set the reaction priorities based on dependency analysis.
     * @param reactor The reactor on which to do this.
     * @param builder Where to write the code.
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
    public static String deferredConnectInputsToOutputs(
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
                
                // NOTE: For federated execution, dst.parent should always be contained
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
}
