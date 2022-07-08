/*************
 * Copyright (c) 2021, The University of California at Berkeley.
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

package org.lflang.federated.extensions;

import static org.lflang.ASTUtils.convertToEmptyListIfNull;

import java.io.IOException;
import java.nio.file.Files;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Port;
import org.lflang.lf.VarRef;

/**
 * An extension class to the CGenerator that enables certain federated
 * functionalities. Currently, this class offers the following features:
 * 
 * - Allocating and initializing C structures for federated communication -
 * Creating status field for network input ports that help the receiver logic in
 * federate.c communicate the status of a network input port with network input
 * control reactions.
 * 
 * @author Soroush Bateni {soroush@utdallas.edu}
 *
 */
public class CExtension implements FedTargetExtension {

    @Override
    public void initializeTargetConfig(FederateInstance federate, FedFileConfig fileConfig, TargetConfig targetConfig, ErrorReporter errorReporter) throws IOException {
        if(GeneratorUtils.isHostWindows()) {
            errorReporter.reportError(
                "Federated LF programs with a C target are currently not supported on Windows. " +
                    "Exiting code generation."
            );
            // Return to avoid compiler errors
            return;
        }

        CExtensionUtils.generateCMakeInclude(fileConfig, targetConfig);
        targetConfig.useCmake = true;


        // Reset the cmake-includes and files, to be repopulated for each federate individually.
        // This is done to enable support for separately
        // adding cmake-includes/files for different federates to prevent linking and mixing
        // all federates' supporting libraries/files together. FIXME: most likely not needed
//        targetConfig.cmakeIncludes.clear();
//        targetConfig.cmakeIncludesWithoutPath.clear();
//        targetConfig.fileNames.clear();
//        targetConfig.filesNamesWithoutPath.clear();

        // Re-apply the cmake-include target property of the federate's .lf file.
        var target = GeneratorUtils.findTarget(federate.instantiation.getReactorClass().eResource());
        if (target.getConfig() != null) {
            // Update the cmake-include
            TargetProperty.updateOne(
                targetConfig,
                TargetProperty.CMAKE_INCLUDE,
                convertToEmptyListIfNull(target.getConfig().getPairs()),
                errorReporter
            );
            // Update the files
            TargetProperty.updateOne(
                targetConfig,
                TargetProperty.FILES,
                convertToEmptyListIfNull(target.getConfig().getPairs()),
                errorReporter
            );
        }
        // Enable clock synchronization if the federate
        // is not local and clock-sync is enabled
        CExtensionUtils.initializeClockSynchronization();
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param connection FIXME
     * @param type FIXME
     * @param coordinationType The coordination type
     * @param errorReporter
     */
    public String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    ) {
        CTypes types = new CTypes(errorReporter);
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
        var receiveRef = CUtil.portRefInReaction(receivingPort, connection.getDstBank(), connection.getDstChannel());
        var result = new CodeBuilder();
        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER);
        // Transfer the physical time of arrival from the action to the port
        result.pr(receiveRef+"->physical_time_of_arrival = self->_lf__"+action.getName()+".physical_time_of_arrival;");
        if (coordinationType == CoordinationType.DECENTRALIZED && !connection.getDefinition().isPhysical()) {
            // Transfer the intended tag.
            result.pr(receiveRef+"->intended_tag = self->_lf__"+action.getName()+".intended_tag;\n");
        }

        deserialize(action, receivingPort, connection, type, receiveRef, result, errorReporter);
        return result.toString();
    }

    /**
     * FIXME
     * @param action
     * @param receivingPort
     * @param connection
     * @param type
     * @param receiveRef
     * @param result
     * @param errorReporter
     */
    protected void deserialize(
        Action action,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        String receiveRef,
        CodeBuilder result,
        ErrorReporter errorReporter
    ) {
        CTypes types = new CTypes(errorReporter);
        var value = "";
        switch (connection.getSerializer()) {
        case NATIVE: {
            // NOTE: Docs say that malloc'd char* is freed on conclusion of the time step.
            // So passing it downstream should be OK.
            value = action.getName()+"->value";
            if (CUtil.isTokenType(type, types)) {
                result.pr("lf_set_token("+ receiveRef +", "+ action.getName()+"->token);");
            } else {
                result.pr("lf_set("+ receiveRef +", "+value+");");
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
            } else if (CExtensionUtils.isSharedPtrType(portType, types)) {
                var matcher = CExtensionUtils.sharedPointerVariable.matcher(portTypeStr);
                if (matcher.find()) {
                    portTypeStr = matcher.group("type");
                }
            }
            var ROSDeserializer = new FedROS2CPPSerialization();
            value = FedROS2CPPSerialization.deserializedVarName;
            result.pr(
                ROSDeserializer.generateNetworkDeserializerCode(
                    "self->_lf__"+ action.getName(),
                    portTypeStr
                )
            );
            if (CExtensionUtils.isSharedPtrType(portType, types)) {
                result.pr("auto msg_shared_ptr = std::make_shared<"+portTypeStr+">("+value+");");
                result.pr("lf_set("+ receiveRef +", msg_shared_ptr);");
            } else {
                result.pr("lf_set("+ receiveRef +", std::move("+value+"));");
            }
            break;
        }
        }
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The variable reference to the destination port.
     * @param connection
     * @param type
     * @param coordinationType
     * @param errorReporter FIXME
     */
    public String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    ) {
        var sendRef = CUtil.portRefInReaction(sendingPort, connection.getSrcBank(), connection.getSrcChannel());
        var receiveRef = ASTUtils.generateVarRef(receivingPort); // Used for comments only, so no need for bank/multiport index.
        var result = new CodeBuilder();
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        int receivingPortID = connection.getDstFederate().networkMessageActions.size();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");

        result.pr("// Sending from " + sendRef + " in federate "
                      + connection.getSrcFederate().name + " to " + receiveRef
                      + " in federate " + connection.getDstFederate().name);

        // In case sendRef is a multiport or is in a bank, this reaction will be triggered when any channel or bank index of sendRef is present
        // ex. if a.out[i] is present, the entire output a.out is triggered.
        if (connection.getSrcBank() != -1 || connection.getSrcChannel() != -1) {
            result.pr("if (!"+sendRef+"->is_present) return;");
        }

        // If the connection is physical and the receiving federate is remote, send it directly on a socket.
        // If the connection is logical and the coordination mode is centralized, send via RTI.
        // If the connection is logical and the coordination mode is decentralized, send directly
        String messageType;
        // Name of the next immediate destination of this message
        var next_destination_name = "\"federate "+connection.getDstFederate().id+"\"";

        // Get the delay literal
        String additionalDelayString = CExtensionUtils.getNetworkDelayLiteral(connection.getDefinition().getDelay());

        if (connection.getDefinition().isPhysical()) {
            messageType = "MSG_TYPE_P2P_MESSAGE";
        } else if (coordinationType == CoordinationType.DECENTRALIZED) {
            messageType = "MSG_TYPE_P2P_TAGGED_MESSAGE";
        } else {
            // Logical connection
            // Send the message via rti
            messageType = "MSG_TYPE_TAGGED_MESSAGE";
            next_destination_name = "\"federate "+connection.getDstFederate().id+" via the RTI\"";
        }


        String sendingFunction = "send_timed_message";
        String commonArgs = String.join(", ",
                                        additionalDelayString,
                                        messageType,
                                        receivingPortID + "",
                                        connection.getDstFederate().id + "",
                                        next_destination_name,
                                        "message_length"
        );
        if (connection.getDefinition().isPhysical()) {
            // Messages going on a physical connection do not
            // carry a timestamp or require the delay;
            sendingFunction = "send_message";
            commonArgs = messageType+", "+receivingPortID+", "+connection.getDstFederate().id+", "+next_destination_name+", message_length";
        }

        serializeAndSend(
            connection,
            type,
            sendRef,
            result,
            sendingFunction,
            commonArgs,
            errorReporter
        );
        return result.toString();
    }

    /**
     * FIXME
     * @param connection
     * @param type
     * @param sendRef
     * @param result
     * @param sendingFunction
     * @param commonArgs
     * @param errorReporter
     */
    protected void serializeAndSend(
        FedConnectionInstance connection,
        InferredType type,
        String sendRef,
        CodeBuilder result,
        String sendingFunction,
        String commonArgs,
        ErrorReporter errorReporter
    ) {
        CTypes types = new CTypes(errorReporter);
        var lengthExpression = "";
        var pointerExpression = "";
        switch (connection.getSerializer()) {
        case NATIVE: {
            // Handle native types.
            if (CUtil.isTokenType(type, types)) {
                // NOTE: Transporting token types this way is likely to only work if the sender and receiver
                // both have the same endianness. Otherwise, you have to use protobufs or some other serialization scheme.
                result.pr("size_t message_length = "+ sendRef +"->token->length * "+ sendRef
                              +"->token->element_size;");
                result.pr(sendingFunction +"("+ commonArgs +", (unsigned char*) "+ sendRef
                              +"->value);");
            } else {
                // string types need to be dealt with specially because they are hidden pointers.
                // void type is odd, but it avoids generating non-standard expression sizeof(void),
                // which some compilers reject.
                lengthExpression = "sizeof("+ types.getTargetType(type)+")";
                pointerExpression = "(unsigned char*)&"+ sendRef +"->value";
                var targetType = types.getTargetType(type);
                if (targetType.equals("string")) {
                    lengthExpression = "strlen("+ sendRef +"->value) + 1";
                    pointerExpression = "(unsigned char*) "+ sendRef +"->value";
                } else if (targetType.equals("void")) {
                    lengthExpression = "0";
                }
                result.pr("size_t message_length = "+lengthExpression+";");
                result.pr(
                    sendingFunction +"("+ commonArgs +", "+pointerExpression+");");
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
            } else if (CExtensionUtils.isSharedPtrType(type, types)) {
                var matcher = CExtensionUtils.sharedPointerVariable.matcher(typeStr);
                if (matcher.find()) {
                    typeStr = matcher.group("type");
                }
            }
            var ROSSerializer = new FedROS2CPPSerialization();
            lengthExpression = ROSSerializer.serializedBufferLength();
            pointerExpression = ROSSerializer.seializedBufferVar();
            result.pr(
                ROSSerializer.generateNetworkSerializerCode(variableToSerialize, typeStr, CExtensionUtils.isSharedPtrType(type, types))
            );
            result.pr("size_t message_length = "+lengthExpression+";");
            result.pr(sendingFunction +"("+ commonArgs +", "+pointerExpression+");");
            break;
        }

        }
    }

    /**
     * Generate code for the body of a reaction that decides whether the trigger for the given
     * port is going to be present or absent for the current logical time.
     * This reaction is put just before the first reaction that is triggered by the network
     * input port "port" or has it in its sources. If there are only connections to contained
     * reactors, in the top-level reactor.
     *
     * @param receivingPortID The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     */
    public String generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP,
        CoordinationType coordination
    ) {
        // Store the code
        var result = new CodeBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        result.pr("interval_t max_STP = 0LL;");

        // Find the maximum STP for decentralized coordination
        if(coordination == CoordinationType.DECENTRALIZED) {
            result.pr("max_STP = "+ GeneratorBase.timeInTargetLanguage(maxSTP)+";");
        }
        result.pr("// Wait until the port status is known");
        result.pr("wait_until_port_status_known("+receivingPortID+", max_STP);");
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     *
     * @oaram srcOutputPort FIXME
     * @param connection FIXME
     */
    public String generateNetworkOutputControlReactionBody(
        VarRef srcOutputPort,
        FedConnectionInstance connection
    ) {
        // Store the code
        var result = new CodeBuilder();
        // The ID of the receiving port (rightPort) is the position
        // of the networkAction (see below) in this list.
        int receivingPortID = connection.getDstFederate().networkMessageActions.size();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        var sendRef = CUtil.portRefInReaction(srcOutputPort, connection.getSrcBank(), connection.getSrcChannel());
        // Get the delay literal
        var additionalDelayString = CExtensionUtils.getNetworkDelayLiteral(connection.getDefinition().getDelay());
        result.pr(String.join("\n",
                              "// If the output port has not been lf_set for the current logical time,",
                              "// send an ABSENT message to the receiving federate            ",
                              "LF_PRINT_LOG(\"Contemplating whether to send port \"",
                              "          \"absent for port %d to federate %d.\", ",
                              "          "+receivingPortID+", "+connection.getDstFederate().id+");",
                              "if ("+sendRef+" == NULL || !"+sendRef+"->is_present) {",
                              "    // The output port is NULL or it is not present.",
                              "    send_port_absent_to_federate("+additionalDelayString+", "+receivingPortID+", "+connection.getDstFederate().id+");",
                              "}"
        ));
        return result.toString();
    }


    public String getNetworkBufferType() {
        return "uint8_t*";
    }

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @return
     */
    @Override
    public String generatePreamble(FederateInstance federate) {
//        if (!IterableExtensions.isNullOrEmpty(targetConfig.protoFiles)) {
//            // Enable support for proto serialization
//            enabledSerializers.add(SupportedSerializers.PROTO);
//        }
//        for (SupportedSerializers serializer : enabledSerializers) {
//            switch (serializer) {
//            case NATIVE: {
//                // No need to do anything at this point.
//                break;
//            }
//            case PROTO: {
//                // Handle .proto files.
//                for (String file : targetConfig.protoFiles) {
//                    this.processProtoFile(file, cancelIndicator);
//                    var dotIndex = file.lastIndexOf(".");
//                    var rootFilename = file;
//                    if (dotIndex > 0) {
//                        rootFilename = file.substring(0, dotIndex);
//                    }
//                    code.pr("#include " + addDoubleQuotes(rootFilename + ".pb-c.h"));
//                }
//                break;
//            }
//            case ROS2: {
//                var ROSSerializer = new FedROS2CPPSerialization();
//                code.pr(ROSSerializer.generatePreambleForSupport().toString());
//                cMakeExtras = String.join("\n",
//                                          cMakeExtras,
//                                          ROSSerializer.generateCompilerExtensionForSupport()
//                );
//                break;
//            }
//            }
//        }
        return
        """
        preamble {=
            %s
        =}""".formatted("");
    }

}
