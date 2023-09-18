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

import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;

/**
 * An extension class to the CGenerator that enables certain federated functionalities.
 *
 * @author {Soroush Bateni <soroush@berkeley.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 * @author {Billy Bao <billybao@berkeley.edu>}
 */
public class CExtension implements FedTargetExtension {

  @Override
  public void initializeTargetConfig(
      LFGeneratorContext context,
      int numOfFederates,
      FederateInstance federate,
      FedFileConfig fileConfig,
      MessageReporter messageReporter,
      RtiConfig rtiConfig)
      throws IOException {

    CExtensionUtils.handleCompileDefinitions(federate, numOfFederates, rtiConfig, messageReporter);

    generateCMakeInclude(federate, fileConfig);

    federate.targetConfig.keepalive = true;
    federate.targetConfig.setByUser.add(TargetProperty.KEEPALIVE);

    // If there are federates, copy the required files for that.
    // Also, create the RTI C file and the launcher script.
    // Handle target parameters.
    // If the program is federated, then ensure that threading is enabled.
    federate.targetConfig.threading = true;
    federate.targetConfig.setByUser.add(TargetProperty.THREADING);

    // Include the fed setup file for this federate in the target property
    federate.targetConfig.fedSetupPreamble = getPreamblePath(federate);
    federate.targetConfig.setByUser.add(TargetProperty.FED_SETUP);
  }

  /** Generate a cmake-include file for {@code federate} if needed. */
  protected void generateCMakeInclude(FederateInstance federate, FedFileConfig fileConfig)
      throws IOException {
    CExtensionUtils.generateCMakeInclude(federate, fileConfig);
  }

  /**
   * Generate code for the body of a reaction that handles the action that is triggered by receiving
   * a message from a remote federate.
   *
   * @param action The action.
   * @param sendingPort The output port providing the data to send.
   * @param receivingPort The ID of the destination port.
   * @param connection The federated connection being lowered.
   * @param type The type of the data conveyed by the port.
   * @param coordinationType The coordination type
   */
  public String generateNetworkReceiverBody(
      Action action,
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationType coordinationType,
      MessageReporter messageReporter) {
    var receiveRef =
        CUtil.portRefInReaction(receivingPort, connection.getDstBank(), connection.getDstChannel());
    var result = new CodeBuilder();
    // Transfer the physical time of arrival from the action to the port
    result.pr(
        receiveRef
            + "->physical_time_of_arrival = self->_lf__"
            + action.getName()
            + ".physical_time_of_arrival;");
    if (coordinationType == CoordinationType.DECENTRALIZED
        && !connection.getDefinition().isPhysical()) {
      // Transfer the intended tag.
      result.pr(
          receiveRef + "->intended_tag = self->_lf__" + action.getName() + ".intended_tag;\n");
    }

    deserialize(action, receivingPort, connection, type, receiveRef, result, messageReporter);
    return result.toString();
  }

  /**
   * Generate code to deserialize a message received over the network.
   *
   * @param action The network action that is mapped to the {@code receivingPort}
   * @param receivingPort The receiving port
   * @param connection The connection used to receive the message
   * @param type Type for the port
   * @param receiveRef A target language reference to the receiving port
   * @param result Used to put generated code in
   * @param messageReporter Used to report errors, if any
   */
  protected void deserialize(
      Action action,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      String receiveRef,
      CodeBuilder result,
      MessageReporter messageReporter) {
    CTypes types = new CTypes();
    // Adjust the type of the action and the receivingPort.
    // If it is "string", then change it to "char*".
    // This string is dynamically allocated, and type 'string' is to be
    // used only for statically allocated strings. This would force the
    // CGenerator to treat the port and action as token types.
    if (types.getTargetType(action).equals("string")) {
      action.getType().setCode(null);
      action.getType().setId("char*");
    }
    if (types.getTargetType((Port) receivingPort.getVariable()).equals("string")) {
      ((Port) receivingPort.getVariable()).getType().setCode(null);
      ((Port) receivingPort.getVariable()).getType().setId("char*");
    }
    var value = "";
    switch (connection.getSerializer()) {
      case NATIVE -> {
        // NOTE: Docs say that malloc'd char* is freed on conclusion of the time step.
        // So passing it downstream should be OK.
        value = action.getName() + "->value";
        if (CUtil.isTokenType(type, types)) {
          result.pr("lf_set_token(" + receiveRef + ", " + action.getName() + "->token);");
        } else {
          result.pr("lf_set(" + receiveRef + ", " + value + ");");
        }
      }
      case PROTO -> throw new UnsupportedOperationException(
          "Protobuf serialization is not supported yet.");
      case ROS2 -> {
        var portType = ASTUtils.getInferredType(((Port) receivingPort.getVariable()));
        var portTypeStr = types.getTargetType(portType);
        if (CUtil.isTokenType(portType, types)) {
          throw new UnsupportedOperationException(
              "Cannot handle ROS serialization when ports are pointers.");
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
                "self->_lf__" + action.getName(), portTypeStr));
        if (CExtensionUtils.isSharedPtrType(portType, types)) {
          result.pr("auto msg_shared_ptr = std::make_shared<" + portTypeStr + ">(" + value + ");");
          result.pr("lf_set(" + receiveRef + ", msg_shared_ptr);");
        } else {
          result.pr("lf_set(" + receiveRef + ", std::move(" + value + "));");
        }
      }
    }
  }

  @Override
  public String outputInitializationBody() {
    return """
    extern reaction_t* port_absent_reaction[];
    void enqueue_port_absent_reactions(environment_t*);
    LF_PRINT_DEBUG("Adding network port absent reaction to table.");
    port_absent_reaction[SENDERINDEXPARAMETER] = &self->_lf__reaction_2;
    LF_PRINT_DEBUG("Added network output control reaction to table. Enqueueing it...");
    enqueue_port_absent_reactions(self->base.environment);
    """;
  }

  @Override
  public String inputInitializationBody() {
    return "self->_lf__reaction_1.is_an_input_reaction = true;\n";
  }

  @Override
  public void addSenderIndexParameter(Reactor sender) {
    var tp = LfFactory.eINSTANCE.createTypeParm();
    tp.setLiteral("SENDERINDEXPARAMETER");
    sender.getTypeParms().add(tp);
  }

  @Override
  public void supplySenderIndexParameter(Instantiation inst, int idx) {
    var senderIndexParameter = LfFactory.eINSTANCE.createType();
    var c = LfFactory.eINSTANCE.createCode();
    c.setBody(String.valueOf(idx));
    senderIndexParameter.setCode(c);
    inst.getTypeArgs().add(senderIndexParameter);
  }

  /**
   * Generate code for the body of a reaction that handles an output that is to be sent over the
   * network.
   *
   * @param sendingPort The output port providing the data to send.
   * @param receivingPort The variable reference to the destination port.
   * @param connection The federated connection being lowered.
   * @param type The type of the data conveyed by the connection.
   * @param coordinationType Centralized or decentralized.
   */
  public String generateNetworkSenderBody(
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationType coordinationType,
      MessageReporter messageReporter) {
    var sendRef =
        CUtil.portRefInReaction(sendingPort, connection.getSrcBank(), connection.getSrcChannel());
    var receiveRef =
        ASTUtils.generateVarRef(
            receivingPort); // Used for comments only, so no need for bank/multiport index.
    var result = new CodeBuilder();
    // The ID of the receiving port (rightPort) is the position
    // of the action in this list.
    int receivingPortID = connection.getDstFederate().networkMessageActions.size();

    result.pr(
        "// Sending from "
            + sendRef
            + " in federate "
            + connection.getSrcFederate().name
            + " to "
            + receiveRef
            + " in federate "
            + connection.getDstFederate().name);

    // In case sendRef is a multiport or is in a bank, this reaction will be triggered when any
    // channel or bank index of sendRef is present
    // ex. if a.out[i] is present, the entire output a.out is triggered.
    if (connection.getSrcBank() != -1 || connection.getSrcChannel() != -1) {
      result.pr("if (!" + sendRef + "->is_present) {");
      if (connection.getSrcFederate().targetConfig.target == Target.Python)
        result.pr("PyGILState_Release(gstate);");
      result.pr("return;");
      result.pr("}");
    }

    // If the connection is physical and the receiving federate is remote, send it directly on a
    // socket.
    // If the connection is logical and the coordination mode is centralized, send via RTI.
    // If the connection is logical and the coordination mode is decentralized, send directly
    String messageType;
    // Name of the next immediate destination of this message
    var next_destination_name = "\"federate " + connection.getDstFederate().id + "\"";

    // Get the delay literal
    String additionalDelayString =
        CExtensionUtils.getNetworkDelayLiteral(connection.getDefinition().getDelay());

    if (connection.getDefinition().isPhysical()) {
      messageType = "MSG_TYPE_P2P_MESSAGE";
    } else if (coordinationType == CoordinationType.DECENTRALIZED) {
      messageType = "MSG_TYPE_P2P_TAGGED_MESSAGE";
    } else {
      // Logical connection
      // Send the message via rti
      messageType = "MSG_TYPE_TAGGED_MESSAGE";
      next_destination_name = "\"federate " + connection.getDstFederate().id + " via the RTI\"";
    }

    String sendingFunction = "send_timed_message";
    String commonArgs =
        String.join(
            ", ",
            "self->base.environment",
            additionalDelayString,
            messageType,
            receivingPortID + "",
            connection.getDstFederate().id + "",
            next_destination_name,
            "_lf_message_length");
    if (connection.getDefinition().isPhysical()) {
      // Messages going on a physical connection do not
      // carry a timestamp or require the delay;
      sendingFunction = "send_message";
      commonArgs =
          messageType
              + ", "
              + receivingPortID
              + ", "
              + connection.getDstFederate().id
              + ", "
              + next_destination_name
              + ", _lf_message_length";
    }

    serializeAndSend(
        connection, type, sendRef, result, sendingFunction, commonArgs, messageReporter);
    return result.toString();
  }

  /**
   * Generate code for serializing data and sending it over the given connection.
   *
   * @param connection A federated connection.
   * @param type The type of the data sent on the connection.
   * @param sendRef C code representing a reference to the data to be sent.
   * @param result An accumulator of the generated code.
   * @param sendingFunction The name of the function that sends the serialized data.
   * @param commonArgs Arguments passed to {@code sendingFunction} regardless of serialization
   *     method.
   */
  protected void serializeAndSend(
      FedConnectionInstance connection,
      InferredType type,
      String sendRef,
      CodeBuilder result,
      String sendingFunction,
      String commonArgs,
      MessageReporter messageReporter) {
    CTypes types = new CTypes();
    var lengthExpression = "";
    var pointerExpression = "";
    switch (connection.getSerializer()) {
      case NATIVE -> {
        // Handle native types.
        if (CUtil.isTokenType(type, types)) {
          // NOTE: Transporting token types this way is likely to only work if the sender and
          // receiver
          // both have the same endianness. Otherwise, you have to use protobufs or some other
          // serialization scheme.
          result.pr(
              "size_t _lf_message_length = "
                  + sendRef
                  + "->token->length * "
                  + sendRef
                  + "->token->type->element_size;");
          result.pr(
              sendingFunction + "(" + commonArgs + ", (unsigned char*) " + sendRef + "->value);");
        } else {
          // string types need to be dealt with specially because they are hidden pointers.
          // void type is odd, but it avoids generating non-standard expression sizeof(void),
          // which some compilers reject.
          lengthExpression = "sizeof(" + types.getTargetType(type) + ")";
          pointerExpression = "(unsigned char*)&" + sendRef + "->value";
          var targetType = types.getTargetType(type);
          if (targetType.equals("string")) {
            lengthExpression = "strlen(" + sendRef + "->value) + 1";
            pointerExpression = "(unsigned char*) " + sendRef + "->value";
          } else if (targetType.equals("void")) {
            lengthExpression = "0";
          }
          result.pr("size_t _lf_message_length = " + lengthExpression + ";");
          result.pr(sendingFunction + "(" + commonArgs + ", " + pointerExpression + ");");
        }
      }
      case PROTO -> throw new UnsupportedOperationException(
          "Protobuf serialization is not supported yet.");
      case ROS2 -> {
        var typeStr = types.getTargetType(type);
        if (CUtil.isTokenType(type, types)) {
          throw new UnsupportedOperationException(
              "Cannot handle ROS serialization when ports are pointers.");
        } else if (CExtensionUtils.isSharedPtrType(type, types)) {
          var matcher = CExtensionUtils.sharedPointerVariable.matcher(typeStr);
          if (matcher.find()) {
            typeStr = matcher.group("type");
          }
        }
        var ROSSerializer = new FedROS2CPPSerialization();
        lengthExpression = ROSSerializer.serializedBufferLength();
        pointerExpression = ROSSerializer.serializedBufferVar();
        result.pr(
            ROSSerializer.generateNetworkSerializerCode(
                sendRef, typeStr, CExtensionUtils.isSharedPtrType(type, types)));
        result.pr("size_t _lf_message_length = " + lengthExpression + ";");
        result.pr(sendingFunction + "(" + commonArgs + ", " + pointerExpression + ");");
      }
    }
  }

  /**
   * Generate code for the body of a reaction that sends a port status message for the given port if
   * it is absent.
   *
   * @param srcOutputPort A reference to the port that the sender reaction reads from.
   * @param connection The federated connection being lowered.
   */
  public String generatePortAbsentReactionBody(
      VarRef srcOutputPort, FedConnectionInstance connection) {
    // Store the code
    var result = new CodeBuilder();
    // The ID of the receiving port (rightPort) is the position
    // of the networkAction (see below) in this list.
    int receivingPortID = connection.getDstFederate().networkMessageActions.size();
    var sendRef =
        CUtil.portRefInReaction(srcOutputPort, connection.getSrcBank(), connection.getSrcChannel());
    // Get the delay literal
    var additionalDelayString =
        CExtensionUtils.getNetworkDelayLiteral(connection.getDefinition().getDelay());
    result.pr(
        String.join(
            "\n",
            "// If the output port has not been lf_set for the current logical time,",
            "// send an ABSENT message to the receiving federate            ",
            "LF_PRINT_LOG(\"Executing port absent reaction for port %d to federate %d at time"
                + " %lld.\", ",
            "          "
                + receivingPortID
                + ", "
                + connection.getDstFederate().id
                + ", (long long) lf_time_logical_elapsed());",
            "if (" + sendRef + " == NULL || !" + sendRef + "->is_present) {",
            "LF_PRINT_LOG(\"The output port is NULL or it is not present.\");",
            "    send_port_absent_to_federate("
                + "self->base.environment, "
                + additionalDelayString
                + ", "
                + receivingPortID
                + ", "
                + connection.getDstFederate().id
                + ");",
            "}"));
    return result.toString();
  }

  public String getNetworkBufferType() {
    return "uint8_t*";
  }

  /** Put the C preamble in a {@code include/_federate.name + _preamble.h} file. */
  protected final void writePreambleFile(
      FederateInstance federate,
      FedFileConfig fileConfig,
      RtiConfig rtiConfig,
      MessageReporter messageReporter)
      throws IOException {
    String cPreamble = makePreamble(federate, rtiConfig, messageReporter);
    String relPath = getPreamblePath(federate);
    Path fedPreamblePath = fileConfig.getSrcPath().resolve(relPath);
    Files.createDirectories(fedPreamblePath.getParent());
    try (var writer = Files.newBufferedWriter(fedPreamblePath)) {
      writer.write(cPreamble);
    }
  }

  /**
   * Add preamble to a separate file to set up federated execution. Return an empty string since no
   * code generated needs to go in the source.
   */
  @Override
  public String generatePreamble(
      FederateInstance federate,
      FedFileConfig fileConfig,
      RtiConfig rtiConfig,
      MessageReporter messageReporter)
      throws IOException {
    writePreambleFile(federate, fileConfig, rtiConfig, messageReporter);
    var includes = new CodeBuilder();
    includes.pr("""
            #ifdef __cplusplus
            extern "C" {
            #endif""");
    includes.pr("#include \"core/federated/federate.h\"");
    includes.pr("#include \"core/federated/net_common.h\"");
    includes.pr("#include \"core/federated/net_util.h\"");
    includes.pr("#include \"core/federated/clock-sync.h\"");
    includes.pr("#include \"core/threaded/reactor_threaded.h\"");
    includes.pr("#include \"core/utils/util.h\"");
    includes.pr("extern federate_instance_t _fed;");
    includes.pr("""
            #ifdef __cplusplus
            }
            #endif""");
    includes.pr(generateSerializationIncludes(federate, fileConfig));
    return includes.toString();
  }

  /** Generate the preamble to setup federated execution in C. */
  protected String makePreamble(
      FederateInstance federate, RtiConfig rtiConfig, MessageReporter messageReporter) {

    var code = new CodeBuilder();

    code.pr("#include \"core/federated/federate.h\"");
    code.pr("#include \"core/federated/net_common.h\"");
    code.pr("#include \"core/federated/net_util.h\"");
    code.pr("#include \"core/threaded/reactor_threaded.h\"");
    code.pr("#include \"core/utils/util.h\"");
    code.pr("extern federate_instance_t _fed;");

    // Generate function to return a pointer to the action trigger_t
    // that handles incoming network messages destined to the specified
    // port. This will only be used if there are federates.
    int numOfNetworkActions = federate.networkMessageActions.size();
    code.pr(
        """
        lf_action_base_t* _lf_action_table[%1$s];
        size_t _lf_action_table_size = %1$s;
        lf_action_base_t* _lf_zero_delay_action_table[%2$s];
        size_t _lf_zero_delay_action_table_size = %2$s;
        """
            .formatted(numOfNetworkActions, federate.zeroDelayNetworkMessageActions.size()));

    int numOfNetworkReactions = federate.networkReceiverReactions.size();
    code.pr(
        """
        reaction_t* network_input_reactions[%1$s];
        size_t num_network_input_reactions = %1$s;
        """
            .formatted(numOfNetworkReactions));

    int numOfPortAbsentReactions = federate.portAbsentReactions.size();
    code.pr(
        """
        reaction_t* port_absent_reaction[%1$s];  // initialize to null pointers; see C99 6.7.8.10
        size_t num_sender_reactions = %1$s;
        """
            .formatted(numOfPortAbsentReactions));

    int numOfSTAAOffsets = federate.stpOffsets.size();
    code.pr(
        CExtensionUtils.surroundWithIfFederatedDecentralized(
            """
            staa_t* staa_lst[%1$s];
            size_t staa_lst_size = %1$s;
        """
                .formatted(numOfSTAAOffsets)));

    code.pr(generateExecutablePreamble(federate, rtiConfig, messageReporter));

    code.pr(generateSTAAInitialization(federate));

    code.pr(generateInitializeTriggers(federate, messageReporter));

    code.pr(CExtensionUtils.generateFederateNeighborStructure(federate));

    return code.getCode();
  }

  /** Generate preamble code needed for enabled serializers of the federate. */
  protected String generateSerializationIncludes(
      FederateInstance federate, FedFileConfig fileConfig) {
    return CExtensionUtils.generateSerializationIncludes(federate);
  }

  /**
   * Create a function that initializes necessary triggers for federated execution, which are the
   * triggers for control reactions and references to all network actions (which are triggered upon
   * receiving network messages).
   *
   * @param federate The federate to initialize triggers for.
   * @param messageReporter Used to report errors.
   * @return The generated code for the macro.
   */
  private String generateInitializeTriggers(
      FederateInstance federate, MessageReporter messageReporter) {
    CodeBuilder code = new CodeBuilder();
    // Temporarily change the original federate reactor's name in the AST to
    // the federate's name so that trigger references are accurate.
    var federatedReactor = FedASTUtils.findFederatedReactor(federate.instantiation.eResource());
    var oldFederatedReactorName = federatedReactor.getName();
    federatedReactor.setName(federate.name);
    var main = new ReactorInstance(federatedReactor, messageReporter, -1);
    var initializeTriggersForNetworkActions =
        CExtensionUtils.initializeTriggersForNetworkActions(federate, main);
    if (!initializeTriggersForNetworkActions.isBlank())
      code.pr(initializeTriggersForNetworkActions);
    code.pr("staa_initialization(); \\");
    federatedReactor.setName(oldFederatedReactorName);

    return """
            #define initialize_triggers_for_federate() \\
            do { \\
            %s
            } \\
            while (0)
            """
        .formatted((code.getCode().isBlank() ? "\\" : code.getCode()).indent(4).stripTrailing());
  }

  /** Generate code for an executed preamble. */
  private String generateExecutablePreamble(
      FederateInstance federate, RtiConfig rtiConfig, MessageReporter messageReporter) {
    CodeBuilder code = new CodeBuilder();

    code.pr(generateCodeForPhysicalActions(federate, messageReporter));

    code.pr(generateCodeToInitializeFederate(federate, rtiConfig));
    return """
            void _lf_executable_preamble(environment_t* env) {
            %s
            }
            """
        .formatted(code.toString().indent(4).stripTrailing());
  }

  /** Generate code for an executed preamble. */
  private String generateSTAAInitialization(FederateInstance federate) {
    CodeBuilder code = new CodeBuilder();
    code.pr(
        CExtensionUtils.surroundWithIfFederatedDecentralized(CExtensionUtils.stpStructs(federate)));

    return """
            void staa_initialization() {
            %s
            }
            """
        .formatted(code.toString().indent(4).stripTrailing());
  }

  /**
   * Generate code to initialize the {@code federate}.
   *
   * @param rtiConfig Information about the RTI's deployment.
   * @return The generated code
   */
  private String generateCodeToInitializeFederate(FederateInstance federate, RtiConfig rtiConfig) {
    CodeBuilder code = new CodeBuilder();
    code.pr("// ***** Start initializing the federated execution. */");
    code.pr(
        String.join(
            "\n",
            "// Initialize the socket mutex",
            "lf_mutex_init(&outbound_socket_mutex);",
            "lf_cond_init(&port_status_changed, &env->mutex);",
            CExtensionUtils.surroundWithIfFederatedDecentralized(
                "lf_cond_init(&logical_time_changed, &env->mutex);")));

    // Find the STA (A.K.A. the global STP offset) for this federate.
    if (federate.targetConfig.coordination == CoordinationType.DECENTRALIZED) {
      var reactor = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
      var stpParam =
          reactor.getParameters().stream()
              .filter(
                  param ->
                      param.getName().equalsIgnoreCase("STP_offset")
                          && (param.getType() == null || param.getType().isTime()))
              .findFirst();

      if (stpParam.isPresent()) {
        var globalSTP =
            ASTUtils.initialValue(stpParam.get(), List.of(federate.instantiation)).get(0);
        var globalSTPTV = ASTUtils.getLiteralTimeValue(globalSTP);
        code.pr("lf_set_stp_offset(" + CTypes.getInstance().getTargetTimeExpr(globalSTPTV) + ");");
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
    code.pr("_lf_my_fed_id = " + federate.id + ";");

    // We keep separate record for incoming and outgoing p2p connections to allow incoming traffic
    // to be processed in a separate
    // thread without requiring a mutex lock.
    var numberOfInboundConnections = federate.inboundP2PConnections.size();
    var numberOfOutboundConnections = federate.outboundP2PConnections.size();

    code.pr(
        String.join(
            "\n",
            "_fed.number_of_inbound_p2p_connections = " + numberOfInboundConnections + ";",
            "_fed.number_of_outbound_p2p_connections = " + numberOfOutboundConnections + ";"));
    if (numberOfInboundConnections > 0) {
      code.pr(
          String.join(
              "\n",
              "// Initialize the array of socket for incoming connections to -1.",
              "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
              "    _fed.sockets_for_inbound_p2p_connections[i] = -1;",
              "}"));
    }
    if (numberOfOutboundConnections > 0) {
      code.pr(
          String.join(
              "\n",
              "// Initialize the array of socket for outgoing connections to -1.",
              "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
              "    _fed.sockets_for_outbound_p2p_connections[i] = -1;",
              "}"));
    }

    // If a test clock offset has been specified, insert code to set it here.
    if (federate.targetConfig.clockSyncOptions.testOffset != null) {
      code.pr(
          "lf_set_physical_clock_offset((1 + "
              + federate.id
              + ") * "
              + federate.targetConfig.clockSyncOptions.testOffset.toNanoSeconds()
              + "LL);");
    }

    code.pr(
        String.join(
            "\n",
            "// Connect to the RTI. This sets _fed.socket_TCP_RTI and _lf_rti_socket_UDP.",
            "connect_to_rti("
                + addDoubleQuotes(rtiConfig.getHost())
                + ", "
                + rtiConfig.getPort()
                + ");"));

    // Disable clock synchronization for the federate if it resides on the same host as the RTI,
    // unless that is overridden with the clock-sync-options target property.
    if (CExtensionUtils.clockSyncIsOn(federate, rtiConfig)) {
      code.pr("synchronize_initial_physical_clock_with_rti(_fed.socket_TCP_RTI);");
    }

    if (numberOfInboundConnections > 0) {
      code.pr(
          String.join(
              "\n",
              "// Create a socket server to listen to other federates.",
              "// If a port is specified by the user, that will be used",
              "// as the only possibility for the server. If not, the port",
              "// will start from STARTING_PORT. The function will",
              "// keep incrementing the port until the number of tries reaches PORT_RANGE_LIMIT.",
              "create_server(" + federate.port + ");",
              "// Connect to remote federates for each physical connection.",
              "// This is done in a separate thread because this thread will call",
              "// connect_to_federate for each outbound physical connection at the same",
              "// time that the new thread is listening for such connections for inbound",
              "// physical connections. The thread will live until all connections",
              "// have been established.",
              "lf_thread_create(&_fed.inbound_p2p_handling_thread_id,"
                  + " handle_p2p_connections_from_federates, env);"));
    }

    for (FederateInstance remoteFederate : federate.outboundP2PConnections) {
      code.pr("connect_to_federate(" + remoteFederate.id + ");");
    }
    return code.getCode();
  }

  /**
   * Generate code to handle physical actions in the {@code federate}.
   *
   * @param messageReporter Used to report errors.
   * @return Generated code.
   */
  private String generateCodeForPhysicalActions(
      FederateInstance federate, MessageReporter messageReporter) {
    CodeBuilder code = new CodeBuilder();
    if (federate.targetConfig.coordination.equals(CoordinationType.CENTRALIZED)) {
      // If this program uses centralized coordination then check
      // for outputs that depend on physical actions so that null messages can be
      // sent to the RTI.
      var federateClass = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
      var main =
          new ReactorInstance(
              FedASTUtils.findFederatedReactor(federate.instantiation.eResource()),
              messageReporter,
              1);
      var instance = new ReactorInstance(federateClass, main, messageReporter);
      var outputDelayMap = federate.findOutputsConnectedToPhysicalActions(instance);
      var minDelay = TimeValue.MAX_VALUE;
      Output outputFound = null;
      for (Output output : outputDelayMap.keySet()) {
        var outputDelay = outputDelayMap.get(output);
        if (outputDelay.isEarlierThan(minDelay)) {
          minDelay = outputDelay;
          outputFound = output;
        }
      }
      if (minDelay != TimeValue.MAX_VALUE) {
        // Unless silenced, issue a warning.
        if (federate.targetConfig.coordinationOptions.advance_message_interval == null) {
          String message =
              String.join(
                  "\n",
                  "Found a path from a physical action to output for reactor "
                      + addDoubleQuotes(instance.getName())
                      + ". ",
                  "The amount of delay is " + minDelay + ".",
                  "With centralized coordination, this can result in a large number of messages to"
                      + " the RTI.",
                  "Consider refactoring the code so that the output does not depend on the physical"
                      + " action,",
                  "or consider using decentralized coordination. To silence this warning, set the"
                      + " target",
                  "parameter coordination-options with a value like {advance-message-interval: 10"
                      + " msec}");
          messageReporter.at(outputFound).warning(message);
        }
        code.pr(
            "_fed.min_delay_from_physical_action_to_federate_output = "
                + CTypes.getInstance().getTargetTimeExpr(minDelay)
                + ";");
      }
    }
    return code.getCode();
  }

  private String getPreamblePath(FederateInstance f) {
    return "include" + File.separator + "_" + f.name + "_preamble.h";
  }
}
