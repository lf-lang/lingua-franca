package org.lflang.generator.c;

import org.lflang.federated.FederateInstance;
import org.lflang.federated.PythonGeneratorExtension;
import org.lflang.lf.VarRef;
import org.lflang.lf.Action;
import org.lflang.lf.Delay;
import org.lflang.lf.Port;

import java.util.regex.Pattern;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.ReactionInstance;

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
        if (types.getTargetType(action) == "string") {
            action.getType().setCode(null);
            action.getType().setId("char*");
        }
        if (types.getTargetType((Port) receivingPort.getVariable()) == "string") {
            ((Port) receivingPort.getVariable()).getType().setCode(null);
            ((Port) receivingPort.getVariable()).getType().setId("char*");
        }
        var receiveRef = CUtil.portRefInReaction(receivingPort, receivingBankIndex, receivingChannelIndex);
        var result = new StringBuilder();
        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.append("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        // Transfer the physical time of arrival from the action to the port
        result.append(receiveRef+"->physical_time_of_arrival = self->_lf__"+action.getName()+".physical_time_of_arrival;");
        var value = "";
        switch (serializer) {
            case NATIVE: {
                // NOTE: Docs say that malloc'd char* is freed on conclusion of the time step.
                // So passing it downstream should be OK.
                value = action.getName()+"->value";
                if (CUtil.isTokenType(type, types)) {
                    result.append("SET_TOKEN("+receiveRef+", "+action.getName()+"->token);");
                } else {                        
                    result.append("SET("+receiveRef+", "+value+");");
                }
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
                result.append(
                    ROSDeserializer.generateNetworkDeserializerCode(
                        "self->_lf__"+action.getName(),
                        portTypeStr
                    )
                );
                if (isSharedPtrType(portType, types)) {                                     
                    result.append("auto msg_shared_ptr = std::make_shared<"+portTypeStr+">("+value+");");
                    result.append("SET("+receiveRef+", msg_shared_ptr);");
                } else {                                      
                    result.append("SET("+receiveRef+", std::move("+value+"));");
                }
            }
        }
        return result.toString();
    }
}
