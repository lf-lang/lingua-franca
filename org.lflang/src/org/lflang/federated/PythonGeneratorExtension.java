/*************
 * Copyright (c) 2021, The University of Texas at Dallas.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.federated;

import org.lflang.InferredType;
import org.lflang.ASTUtils;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.FedNativePythonSerialization;
import org.lflang.federated.serialization.FedSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Delay;
import org.lflang.lf.VarRef;

/**
 * An extension class to the PythonGenerator that enables certain federated
 * functionalities.
 * 
 * @author Soroush Bateni {soroush@utdallas.edu}
 *
 */
public class PythonGeneratorExtension {
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
     * @param generator The instance of the PythonGenerator.
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
        CoordinationType coordinationType
    ) { 
        String sendRef = CUtil.portRefInReaction(sendingPort, sendingBankIndex, sendingChannelIndex);
        String receiveRef = ASTUtils.generateVarRef(receivingPort); // Used for comments only, so no need for bank/multiport index.
        StringBuilder result = new StringBuilder();
        result.append("// Sending from " + sendRef + 
                " in federate " + sendingFed.name + " to " + receiveRef + " in federate " + receivingFed.name + "\n");
        // If the connection is physical and the receiving federate is remote, send it directly on a socket.
        // If the connection is logical and the coordination mode is centralized, send via RTI.
        // If the connection is logical and the coordination mode is decentralized, send directly
        String messageType;
        // Name of the next immediate destination of this message
        String next_destination_name = "\"federate " + receivingFed.id + "\"";
        
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
            next_destination_name = "\"federate " + receivingFed.id + " via the RTI\"";
        }
        
        
        String sendingFunction = "send_timed_message";
        String commonArgs = additionalDelayString + ", " 
                   + messageType + ", " 
                   + receivingPortID + ", " 
                   + receivingFed.id + ", " 
                   + next_destination_name + ", " 
                   + "message_length";
        if (isPhysical) {
            // Messages going on a physical connection do not
            // carry a timestamp or require the delay;
            sendingFunction = "send_message";        
            commonArgs = messageType + ", " + receivingPortID + ", " + receivingFed.id + ", "
                   + next_destination_name + ", message_length";
        }
        
        String lengthExpression = "";
        String pointerExpression = "";
        switch (serializer) {
            case NATIVE: {
                var variableToSerialize = sendRef+"->value";
                FedNativePythonSerialization pickler = new FedNativePythonSerialization();
                lengthExpression = pickler.serializedBufferLength();
                pointerExpression = pickler.seializedBufferVar();
                result.append(
                        pickler.generateNetworkSerializerCode(variableToSerialize, null)
                );
                result.append("size_t message_length = "+lengthExpression+";\n");
                result.append(sendingFunction+"("+commonArgs+", "+pointerExpression+");\n");
                break;
            }
            case PROTO: {
                throw new UnsupportedOperationException("Protbuf serialization is not supported yet.");
            }
            case ROS2: {
                throw new UnsupportedOperationException("ROS2 serialization is not supported yet.");
            }
            
        }
        return result.toString();
    }
    
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
     * @param generator The instance of the PythonGenerator.
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
        SupportedSerializers serializer
    ) {

        String receiveRef = CUtil.portRefInReaction(receivingPort, receivingBankIndex, receivingChannelIndex);
        StringBuilder result = new StringBuilder();
      
        // Transfer the physical time of arrival from the action to the port
        result.append(receiveRef+"->physical_time_of_arrival = self->_lf__"+action.getName()+".physical_time_of_arrival;\n");
        
        
        String value = "";
        switch (serializer) {
            case NATIVE: {
                value = action.getName();
                FedNativePythonSerialization pickler = new FedNativePythonSerialization();
                result.append(pickler.generateNetworkDeserializerCode(value, null));
                result.append("SET("+receiveRef+", "+FedSerialization.deserializedVarName+");\n");
                break;
            }
            case PROTO: {
                throw new UnsupportedOperationException("Protbuf serialization is not supported yet.");
            }
            case ROS2: {
                throw new UnsupportedOperationException("ROS2 serialization is not supported yet.");
            }
            
        }
        
        return result.toString();
    }
}
