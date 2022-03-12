package org.lflang.generator.c;

import org.lflang.federated.CGeneratorExtension;
import org.lflang.federated.FederateInstance;
import org.lflang.lf.VarRef;
import org.lflang.lf.Action;
import org.lflang.lf.Delay;
import org.lflang.lf.Port;

import java.util.regex.Pattern;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.TimeValue;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactionInstance;

/**
 * Generates C code to support messaging-related functionalities
 * in federated execution.
 * 
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CNetworkGenerator {
    private static boolean isSharedPtrType(InferredType type, CTypes types) {
        return !type.isUndefined() && sharedPointerVariable.matcher(types.getTargetType(type)).find();
    }
    
    // Regular expression pattern for shared_ptr types.
    static final Pattern sharedPointerVariable = Pattern.compile("^std::shared_ptr<(\\S+)>$");

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @param isPhysical Indicates whether or not the connection is physical
     * @param serializer The serializer used on the connection.
     */
    public static String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        int receivingBankIndex,
        int receivingChannelIndex,
        InferredType type,
        boolean isPhysical,
        SupportedSerializers serializer,
        CTypes types
    ) {
        // Adjust the type of the action and the receivingPort.
        // If it is "string", then change it to "char*".
        // This string is dynamically allocated, and type 'string' is to be
        // used only for statically allocated strings.
        // FIXME: Is the getTargetType method not responsible for generating the desired C code
        //  (e.g., char* rather than string)? If not, what exactly is that method
        //  responsible for? If generateNetworkReceiverBody has different requirements
        //  than those that the method was designed to satisfy, should we use a different
        //  method? The best course of action is not obvious, but we have a pattern of adding
        //  downstream patches to generated strings rather than fixing them at their source.
        if (types.getTargetType(action).equals("string")) {
            action.getType().setCode(null);
            action.getType().setId("char*");
        }
        if (types.getTargetType((Port) receivingPort.getVariable()).equals("string")) {
            ((Port) receivingPort.getVariable()).getType().setCode(null);
            ((Port) receivingPort.getVariable()).getType().setId("char*");
        }
        var receiveRef = CUtil.portRefInReaction(receivingPort, receivingBankIndex, receivingChannelIndex);
        var result = new CodeBuilder();
        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER);
        // Transfer the physical time of arrival from the action to the port
        result.pr(receiveRef+"->physical_time_of_arrival = self->_lf__"+action.getName()+".physical_time_of_arrival;");
        var value = "";
        switch (serializer) {
            case NATIVE: {
                // NOTE: Docs say that malloc'd char* is freed on conclusion of the time step.
                // So passing it downstream should be OK.
                value = action.getName()+"->value";
                if (CUtil.isTokenType(type, types)) {
                    result.pr("SET_TOKEN("+receiveRef+", "+action.getName()+"->token);");
                } else {                        
                    result.pr("SET("+receiveRef+", "+value+");");
                }
                break;
            }
            case PROTO: {
                throw new UnsupportedOperationException("Protobuf serialization is not supported yet.");
            }
            case ROS2: {
                var portType = ASTUtils.getInferredType(((Port) receivingPort.getVariable()));
                var portTypeStr = types.getTargetType(portType);
                if (CUtil.isTokenType(portType, types)) {
                    throw new UnsupportedOperationException("Cannot handle ROS serialization when ports are pointers.");
                } else if (isSharedPtrType(portType, types)) {
                    var matcher = sharedPointerVariable.matcher(portTypeStr);
                    if (matcher.find()) {
                        portTypeStr = matcher.group(1);
                    }
                }
                var ROSDeserializer = new FedROS2CPPSerialization();
                value = FedROS2CPPSerialization.deserializedVarName;
                result.pr(
                    ROSDeserializer.generateNetworkDeserializerCode(
                        "self->_lf__"+action.getName(),
                        portTypeStr
                    )
                );
                if (isSharedPtrType(portType, types)) {                                     
                    result.pr("auto msg_shared_ptr = std::make_shared<"+portTypeStr+">("+value+");");
                    result.pr("SET("+receiveRef+", msg_shared_ptr);");
                } else {                                      
                    result.pr("SET("+receiveRef+", std::move("+value+"));");
                }
                break;
            }
        }
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The variable reference to the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @param serializer The serializer used on the connection.
     */
    public static String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID, 
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay,
        SupportedSerializers serializer,
        CTypes types,
        CoordinationType coordinationType
    ) { 
        var sendRef = CUtil.portRefInReaction(sendingPort, sendingBankIndex, sendingChannelIndex);
        var receiveRef = ASTUtils.generateVarRef(receivingPort); // Used for comments only, so no need for bank/multiport index.
        var result = new CodeBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");

        result.pr("// Sending from "+sendRef+" in federate "+sendingFed.name+" to "+receiveRef+" in federate "+receivingFed.name);
        // If the connection is physical and the receiving federate is remote, send it directly on a socket.
        // If the connection is logical and the coordination mode is centralized, send via RTI.
        // If the connection is logical and the coordination mode is decentralized, send directly
        String messageType;
        // Name of the next immediate destination of this message
        var next_destination_name = "\"federate "+receivingFed.id+"\"";
        
        // Get the delay literal
        String additionalDelayString = CGeneratorExtension.getNetworkDelayLiteral(delay);
        
        if (isPhysical) {
            messageType = "MSG_TYPE_P2P_MESSAGE";
        } else if (coordinationType == CoordinationType.DECENTRALIZED) {
            messageType = "MSG_TYPE_P2P_TAGGED_MESSAGE";
        } else {
            // Logical connection
            // Send the message via rti
            messageType = "MSG_TYPE_TAGGED_MESSAGE";
            next_destination_name = "\"federate "+receivingFed.id+" via the RTI\"";
        }
        
        
        String sendingFunction = "send_timed_message";
        String commonArgs = String.join(", ", 
                   additionalDelayString, 
                   messageType,
                   receivingPortID + "",
                   receivingFed.id + "",
                   next_destination_name,
                   "message_length"
        );
        if (isPhysical) {
            // Messages going on a physical connection do not
            // carry a timestamp or require the delay;
            sendingFunction = "send_message";
            commonArgs = messageType+", "+receivingPortID+", "+receivingFed.id+", "+next_destination_name+", message_length";
        }
        
        var lengthExpression = "";
        var pointerExpression = "";
        switch (serializer) {
            case NATIVE: {
                // Handle native types.
                if (CUtil.isTokenType(type, types)) {
                    // NOTE: Transporting token types this way is likely to only work if the sender and receiver
                    // both have the same endianness. Otherwise, you have to use protobufs or some other serialization scheme.
                    result.pr("size_t message_length = "+sendRef+"->token->length * "+sendRef+"->token->element_size;");
                    result.pr(sendingFunction+"("+commonArgs+", (unsigned char*) "+sendRef+"->value);");
                } else {
                    // string types need to be dealt with specially because they are hidden pointers.
                    // void type is odd, but it avoids generating non-standard expression sizeof(void),
                    // which some compilers reject.
                    lengthExpression = "sizeof("+types.getTargetType(type)+")";
                    pointerExpression = "(unsigned char*)&"+sendRef+"->value";
                    var targetType = types.getTargetType(type);
                    if (targetType.equals("string")) {
                        lengthExpression = "strlen("+sendRef+"->value) + 1";
                        pointerExpression = "(unsigned char*) "+sendRef+"->value";
                    } else if (targetType.equals("void")) {
                        lengthExpression = "0";
                    }
                    result.pr("size_t message_length = "+lengthExpression+";");
                    result.pr(sendingFunction+"("+commonArgs+", "+pointerExpression+");");
                }
                break;
            }
            case PROTO: {
                throw new UnsupportedOperationException("Protobuf serialization is not supported yet.");
            }
            case ROS2: {
                var variableToSerialize = sendRef;
                var typeStr = types.getTargetType(type);
                if (CUtil.isTokenType(type, types)) {
                    throw new UnsupportedOperationException("Cannot handle ROS serialization when ports are pointers.");
                } else if (isSharedPtrType(type, types)) {
                    var matcher = sharedPointerVariable.matcher(typeStr);
                    if (matcher.find()) {
                        typeStr = matcher.group(1);
                    }
                }
                var ROSSerializer = new FedROS2CPPSerialization();
                lengthExpression = ROSSerializer.serializedBufferLength();
                pointerExpression = ROSSerializer.seializedBufferVar();
                result.pr(
                    ROSSerializer.generateNetworkSerializerCode(variableToSerialize, typeStr, isSharedPtrType(type, types))
                );
                result.pr("size_t message_length = "+lengthExpression+";");
                result.pr(sendingFunction+"("+commonArgs+", "+pointerExpression+");");
                break;
            }
            
        }
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that decides whether the trigger for the given
     * port is going to be present or absent for the current logical time.
     * This reaction is put just before the first reaction that is triggered by the network
     * input port "port" or has it in its sources. If there are only connections to contained 
     * reactors, in the top-level reactor.
     * 
     * @param port The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     */
    public static String generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP,
        boolean isFederatedAndDecentralized
    ) {
        // Store the code
        var result = new CodeBuilder();
        
        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        result.pr("interval_t max_STP = 0LL;");
        
        // Find the maximum STP for decentralized coordination
        if(isFederatedAndDecentralized) {
            result.pr("max_STP = "+GeneratorBase.timeInTargetLanguage(maxSTP)+";");
        }
        result.pr("// Wait until the port status is known");
        result.pr("wait_until_port_status_known("+receivingPortID+", max_STP);");
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     * 
     * @param port The port to generate the control reaction for
     * @param portID The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex The bank index of the sending federate, if it is in a bank.
     * @param sendingChannelIndex The channel if a multiport
     * @param delay The delay value imposed on the connection using after
     */
    public static String generateNetworkOutputControlReactionBody(
        VarRef port,
        int portID,
        int receivingFederateID,
        int sendingBankIndex,
        int sendingChannelIndex,
        Delay delay
    ) {
        // Store the code
        var result = new CodeBuilder();
        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        var sendRef = CUtil.portRefInReaction(port, sendingBankIndex, sendingChannelIndex);
        // Get the delay literal
        var additionalDelayString = CGeneratorExtension.getNetworkDelayLiteral(delay);
        result.pr(String.join("\n", 
            "// If the output port has not been SET for the current logical time,",
            "// send an ABSENT message to the receiving federate            ",
            "LOG_PRINT(\"Contemplating whether to send port \"",
            "          \"absent for port %d to federate %d.\", ",
            "          "+portID+", "+receivingFederateID+");",
            "if (!"+sendRef+"->is_present) {",
            "    send_port_absent_to_federate("+additionalDelayString+", "+portID+", "+receivingFederateID+");",
            "}"
        ));
        return result.toString();
    }
}
