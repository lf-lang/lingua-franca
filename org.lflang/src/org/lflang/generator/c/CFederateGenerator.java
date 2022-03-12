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

/**
 * Generate code for federate related functionality
 * 
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
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
}
