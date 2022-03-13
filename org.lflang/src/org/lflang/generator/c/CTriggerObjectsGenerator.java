package org.lflang.generator.c;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;

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
}
