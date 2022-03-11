package org.lflang.generator.c;

import java.util.Map;

import org.lflang.ASTUtils;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Delay;

public class CFederateGenerator {
    /**
     * Generate code that sends the neighbor structure message to the RTI.
     * @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
     * 
     * @param federate The federate that is sending its neighbor structure
     */
    public static String generateFederateNeighborStructure(FederateInstance federate) {
        var code = new CodeBuilder();
        code.pr(String.join("\n", 
            "/**",
            "* Generated function that sends information about connections between this federate and",
            "* other federates where messages are routed through the RTI. Currently, this",
            "* only includes logical connections when the coordination is centralized. This",
            "* information is needed for the RTI to perform the centralized coordination.",
            "* @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h",
            "*/",
            "void send_neighbor_structure_to_RTI(int rti_socket) {"
        ));
        code.indent();
        // Initialize the array of information about the federate's immediate upstream
        // and downstream relayed (through the RTI) logical connections, to send to the
        // RTI.
        code.pr(String.join("\n", 
                "interval_t candidate_tmp;",
                "size_t buffer_size = 1 + 8 + ",
                "                "+federate.dependsOn.keySet().size()+" * ( sizeof(uint16_t) + sizeof(int64_t) ) +",
                "                "+federate.sendsTo.keySet().size()+" * sizeof(uint16_t);",
                "unsigned char buffer_to_send[buffer_size];",
                "",
                "size_t message_head = 0;",
                "buffer_to_send[message_head] = MSG_TYPE_NEIGHBOR_STRUCTURE;",
                "message_head++;",
                "encode_int32((int32_t)"+federate.dependsOn.keySet().size()+", &(buffer_to_send[message_head]));",
                "message_head+=sizeof(int32_t);",
                "encode_int32((int32_t)"+federate.sendsTo.keySet().size()+", &(buffer_to_send[message_head]));",
                "message_head+=sizeof(int32_t);"
        ));

        if (!federate.dependsOn.keySet().isEmpty()) {
            // Next, populate these arrays.
            // Find the minimum delay in the process.
            // FIXME: Zero delay is not really the same as a microstep delay.
            for (FederateInstance upstreamFederate : federate.dependsOn.keySet()) {
                code.pr(String.join("\n", 
                    "encode_uint16((uint16_t)"+upstreamFederate.id+", &(buffer_to_send[message_head]));",
                    "message_head += sizeof(uint16_t);"
                ));
                // The minimum delay calculation needs to be made in the C code because it
                // may depend on parameter values.
                // FIXME: These would have to be top-level parameters, which don't really
                // have any support yet. Ideally, they could be overridden on the command line.
                // When that is done, they will need to be in scope here.
                var delays = federate.dependsOn.get(upstreamFederate);
                if (delays != null) {
                    // There is at least one delay, so find the minimum.
                    // If there is no delay at all, this is encoded as NEVER.
                    code.pr("candidate_tmp = FOREVER;");
                    for (Delay delay : delays) {
                        if (delay == null) {
                            // Use NEVER to encode no delay at all.
                            code.pr("candidate_tmp = NEVER;");
                        } else {
                            var delayTime = GeneratorBase.getTargetTime(delay);
                            if (delay.getParameter() != null) {
                                // The delay is given as a parameter reference. Find its value.
                                delayTime = GeneratorBase.timeInTargetLanguage(ASTUtils.getDefaultAsTimeValue(delay.getParameter()));
                            }
                            code.pr(String.join("\n",
                                "if ("+delayTime+" < candidate_tmp) {",
                                "    candidate_tmp = "+delayTime+";",
                                "}"
                            ));
                        }
                    }
                    code.pr(String.join("\n",
                        "encode_int64((int64_t)candidate_tmp, &(buffer_to_send[message_head]));",
                        "message_head += sizeof(int64_t);"
                    ));
                } else {
                    // Use NEVER to encode no delay at all.
                    code.pr(String.join("\n", 
                        "encode_int64(NEVER, &(buffer_to_send[message_head]));",
                        "message_head += sizeof(int64_t);"
                    ));
                }
            }
        }

        // Next, set up the downstream array.
        if (!federate.sendsTo.keySet().isEmpty()) {
            // Next, populate the array.
            // Find the minimum delay in the process.
            // FIXME: Zero delay is not really the same as a microstep delay.
            for (FederateInstance downstreamFederate : federate.sendsTo.keySet()) {
                code.pr(String.join("\n", 
                    "encode_uint16("+downstreamFederate.id+", &(buffer_to_send[message_head]));",
                    "message_head += sizeof(uint16_t);"
                ));
            }
        }
        code.pr(String.join("\n", 
            "write_to_socket_errexit(",
            "    rti_socket, ",
            "    buffer_size,",
            "    buffer_to_send,",
            "    \"Failed to send the neighbor structure message to the RTI.\"",
            ");"
        ));
        code.unindent();
        code.pr("}");
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
