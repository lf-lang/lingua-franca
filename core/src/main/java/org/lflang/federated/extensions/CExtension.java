package org.lflang.federated.extensions;

import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.lflang.AttributeUtils;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.generator.ActionInstance;
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
import org.lflang.lf.impl.CodeExprImpl;
import org.lflang.target.Target;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.CoordinationProperty;
import org.lflang.target.property.DNETProperty;
import org.lflang.target.property.FedSetupProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.type.CoordinationModeType.CoordinationMode;

/**
 * An extension class to the CGenerator that enables certain federated functionalities.
 *
 * @author Soroush Bateni
 * @author Hou Seng Wong
 * @author Billy Bao
 * @ingroup Federated
 */
public class CExtension implements FedTargetExtension {

  @Override
  public void initializeTargetConfig(
      LFGeneratorContext context,
      List<String> federateNames,
      FederateInstance federate,
      FederationFileConfig fileConfig,
      MessageReporter messageReporter,
      RtiConfig rtiConfig)
      throws IOException {

    CExtensionUtils.handleCompileDefinitions(federate, federateNames, rtiConfig, messageReporter);

    generateCMakeInclude(federate, fileConfig);

    KeepaliveProperty.INSTANCE.override(federate.targetConfig, true);

    // If there are federates, copy the required files for that.
    // Also, create the RTI C file and the launcher script.
    // Handle target parameters.
    // If the program is federated, then ensure that threading is enabled.
    SingleThreadedProperty.INSTANCE.override(federate.targetConfig, false);

    // Include the fed setup file for this federate in the target property
    FedSetupProperty.INSTANCE.override(federate.targetConfig, getPreamblePath(federate));
  }

  /** Generate a cmake-include file for `federate` if needed. */
  protected void generateCMakeInclude(FederateInstance federate, FederationFileConfig fileConfig)
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
   * @param coordinationMode The coordination type
   */
  public String generateNetworkReceiverBody(
      Action action,
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
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
    if (coordinationMode == CoordinationMode.DECENTRALIZED
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
   * @param action The network action that is mapped to the `receivingPort`
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
    var targetType = types.getTargetType(action);
    var isString = targetType.equals("string");
    if (isString) {
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
        if (CUtil.isTokenType(type) || isString) {
          result.pr("lf_set_token(" + receiveRef + ", " + action.getName() + "->token);");
        } else {
          // Fixed size arrays need to be handled specially because the memory is allocated in the
          // output port.
          if (CUtil.isFixedSizeArrayType(type)) {
            // For fixed size arrays, we need to copy the data from the action to the port.
            result.pr(
                "memcpy("
                    + receiveRef
                    + "->value, "
                    + action.getName()
                    + "->token->value, "
                    + action.getName()
                    + "->length * "
                    + action.getName()
                    + "->type.element_size);");
            result.pr("lf_set_present(" + receiveRef + ");");
          } else {
            result.pr("lf_set(" + receiveRef + ", " + value + ");");
          }
        }
      }
      case PROTO ->
          throw new UnsupportedOperationException("Protobuf serialization is not supported yet.");
      case ROS2 -> {
        var portType = ASTUtils.getInferredType(((Port) receivingPort.getVariable()));
        var portTypeStr = types.getTargetType(portType);
        if (CUtil.isTokenType(portType)) {
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
           void lf_enqueue_port_absent_reactions(environment_t*);
           LF_PRINT_DEBUG("Adding network port absent reaction to table.");
           port_absent_reaction[SENDERINDEXPARAMETER] = &self->_lf__reaction_2;
           LF_PRINT_DEBUG("Added network output control reaction to table. Enqueueing it...");
           lf_enqueue_port_absent_reactions(self->base.environment);
           """;
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
   * @param coordinationMode Centralized or decentralized.
   */
  public String generateNetworkSenderBody(
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
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
    } else if (coordinationMode == CoordinationMode.DECENTRALIZED) {
      messageType = "MSG_TYPE_P2P_TAGGED_MESSAGE";
    } else {
      // Logical connection
      // Send the message via rti
      messageType = "MSG_TYPE_TAGGED_MESSAGE";
      next_destination_name = "\"federate " + connection.getDstFederate().id + " via the RTI\"";
    }

    String sendingFunction = "lf_send_tagged_message";
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
      sendingFunction = "lf_send_message";
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
   * @param commonArgs Arguments passed to `sendingFunction` regardless of serialization
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
        if (CUtil.isTokenType(type)) {
          // NOTE: Transporting token types this way is likely to only work if the sender and
          // receiver both have the same endianness. Otherwise, you have to use protobufs or some
          // other serialization scheme.
          result.pr(
              "size_t _lf_message_length = "
                  + sendRef
                  + "->length * "
                  + sendRef
                  + "->token->type->element_size;");
          result.pr(
              sendingFunction + "(" + commonArgs + ", (unsigned char*) " + sendRef + "->value);");
        } else if (CUtil.isFixedSizeArrayType(type)) {
          result.pr(
              "size_t _lf_message_length = "
                  + CUtil.fixedSizeArrayTypeLength(type)
                  + " * "
                  + sendRef
                  + "->type.element_size;");
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
      case PROTO ->
          throw new UnsupportedOperationException("Protobuf serialization is not supported yet.");
      case ROS2 -> {
        var typeStr = types.getTargetType(type);
        if (CUtil.isTokenType(type) || CUtil.isFixedSizeArrayType(type)) {
          throw new UnsupportedOperationException(
              "Cannot handle ROS serialization when ports are pointers or arrays.");
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
                + "\" PRINTF_TIME \".\", ",
            "          "
                + receivingPortID
                + ", "
                + connection.getDstFederate().id
                + ", (long long) lf_time_logical_elapsed());",
            "if (" + sendRef + " == NULL || !" + sendRef + "->is_present) {",
            "LF_PRINT_LOG(\"The output port is NULL or it is not present.\");",
            "    lf_send_port_absent_to_federate("
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

  /** Put the C preamble in a `include/_federate.name + _preamble.h` file. */
  protected final void writePreambleFile(
      FederateInstance federate,
      FederationFileConfig fileConfig,
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
   * Add preamble to a separate file to set up federated execution. Return an a string containing
   * the #includes that are needed by the federate.
   */
  @Override
  public String generatePreamble(
      FederateInstance federate,
      FederationFileConfig fileConfig,
      RtiConfig rtiConfig,
      MessageReporter messageReporter)
      throws IOException {
    writePreambleFile(federate, fileConfig, rtiConfig, messageReporter);
    var includes = new CodeBuilder();
    includes.pr(
        """
        #ifdef __cplusplus
        extern "C" {
        #endif""");
    includes.pr("#include \"core/federated/federate.h\"");
    includes.pr("#include \"core/federated/network/net_common.h\"");
    includes.pr("#include \"core/federated/network/net_util.h\"");
    includes.pr("#include \"core/federated/network/socket_common.h\"");
    includes.pr("#include \"core/federated/clock-sync.h\"");
    includes.pr("#include \"core/threaded/reactor_threaded.h\"");
    includes.pr("#include \"core/utils/util.h\"");
    includes.pr("extern federate_instance_t _fed;");
    includes.pr(
        """
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
    code.pr("#include \"core/federated/network/net_common.h\"");
    code.pr("#include \"core/federated/network/net_util.h\"");
    code.pr("#include \"core/federated/network/socket_common.h\"");
    code.pr("#include \"core/federated/clock-sync.h\"");
    code.pr("#include \"core/threaded/reactor_threaded.h\"");
    code.pr("#include \"core/utils/util.h\"");
    code.pr("extern federate_instance_t _fed;");

    // Generate function to return a pointer to the action trigger_t
    // that handles incoming network messages destined to the specified
    // port. This will only be used if there are federates.
    int numOfNetworkActions = federate.networkMessageActions.size();
    code.pr(
        """
        interval_t _lf_action_delay_table[%1$s];
        lf_action_base_t* _lf_action_table[%1$s];
        size_t _lf_action_table_size = %1$s;
        lf_action_base_t* _lf_zero_delay_cycle_action_table[%2$s];
        size_t _lf_zero_delay_cycle_action_table_size = %2$s;
        """
            .formatted(numOfNetworkActions, federate.zeroDelayCycleNetworkMessageActions.size()));

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
        size_t num_port_absent_reactions = %1$s;
        """
            .formatted(numOfPortAbsentReactions));

    int numOfSTAAOffsets = federate.staaOffsets.size();
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
      FederateInstance federate, FederationFileConfig fileConfig) {
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

    code.pr(generateCodeToInitializeFederate(federate, rtiConfig, messageReporter));
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
   * Generate code to initialize the `federate`.
   *
   * @param rtiConfig Information about the RTI's deployment.
   * @return The generated code
   */
  private String generateCodeToInitializeFederate(
      FederateInstance federate, RtiConfig rtiConfig, MessageReporter messageReporter) {
    CodeBuilder code = new CodeBuilder();
    code.pr("// ***** Start initializing the federated execution. */");
    code.pr(
        String.join(
            "\n",
            "// Initialize the socket mutexes",
            "lf_mutex_init(&lf_outbound_socket_mutex);",
            "init_shutdown_mutex();",
            "lf_cond_init(&lf_port_status_changed, &env->mutex);"));

    // Find the maxwait (A.K.A. STA, the global STP offset) for this federate.
    if (federate.targetConfig.get(CoordinationProperty.INSTANCE)
        == CoordinationMode.DECENTRALIZED) {
      var reactor = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
      var stpParam =
          ASTUtils.allParameters(reactor).stream()
              .filter(
                  param ->
                      (param.getName().equalsIgnoreCase("STP_offset")
                              || param.getName().equalsIgnoreCase("STA"))
                          && (param.getType() == null || param.getType().isTime()))
              .findFirst();

      if (stpParam.isPresent()) {
        var globalSTP = ASTUtils.initialValue(stpParam.get(), List.of(federate.instantiation));
        var globalSTPTV = ASTUtils.getLiteralTimeValue(globalSTP);
        if (globalSTPTV != null)
          code.pr(
              "lf_set_stp_offset(" + CTypes.getInstance().getTargetTimeExpr(globalSTPTV) + ");");
        else if (globalSTP instanceof CodeExprImpl)
          code.pr("lf_set_stp_offset(" + ((CodeExprImpl) globalSTP).getCode().getBody() + ");");
        else messageReporter.at(stpParam.get().eContainer()).error("Invalid STA offset");
      } else {
        // Check for an annotation on the federate instantiation.
        var maxwait = AttributeUtils.getMaxWait(federate.instantiation);
        if (maxwait != TimeValue.ZERO) {
          code.pr("lf_set_stp_offset(" + CTypes.getInstance().getTargetTimeExpr(maxwait) + ");");
        }
      }
    }

    // Set indicator variables that specify whether the federate has
    // upstream logical connections.
    if (!federate.dependsOn.isEmpty()) {
      code.pr("_fed.has_upstream  = true;");
    }
    if (!federate.sendsTo.isEmpty()) {
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
    code.pr(
        String.join(
            "\n",
            "// Initialize the array of socket for incoming connections to -1.",
            "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
            "    _fed.sockets_for_inbound_p2p_connections[i] = -1;",
            "}"));
    code.pr(
        String.join(
            "\n",
            "// Initialize the array of socket for outgoing connections to -1.",
            "for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {",
            "    _fed.sockets_for_outbound_p2p_connections[i] = -1;",
            "}"));
    var clockSyncOptions = federate.targetConfig.getOrDefault(ClockSyncOptionsProperty.INSTANCE);
    // If a test clock offset has been specified, insert code to set it here.
    if (clockSyncOptions.testOffset != null) {
      code.pr(
          "clock_sync_set_constant_bias((1 + "
              + federate.id
              + ") * "
              + clockSyncOptions.testOffset.toNanoSeconds()
              + "LL);");
    }

    code.pr(
        String.join(
            "\n",
            "// Connect to the RTI. This sets _fed.socket_TCP_RTI and _lf_rti_socket_UDP.",
            "lf_connect_to_rti("
                + addDoubleQuotes(rtiConfig.getHost())
                + ", "
                + rtiConfig.getPort()
                + ");"));

    // Disable clock synchronization for the federate if it resides on the same host as the RTI,
    // unless that is overridden with the clock-sync-options target property.
    if (CExtensionUtils.clockSyncIsOn(federate, rtiConfig)) {
      code.pr("synchronize_initial_physical_clock_with_rti(&_fed.socket_TCP_RTI);");
    }

    if (numberOfInboundConnections > 0) {
      code.pr(
          String.join(
              "\n",
              "// Create a socket server to listen to other federates.",
              "// If a port is specified by the user, that will be used",
              "// as the only possibility for the server. If not, the port",
              "// will be selected by the OS (by specifying port 0).",
              "lf_create_server(" + federate.port + ");",
              "// Connect to remote federates for each physical connection or decentralized"
                  + " connection.",
              "// This is done in a separate thread because this thread will call",
              "// lf_connect_to_federate for each outbound connection at the same",
              "// time that the new thread is listening for such connections for inbound",
              "// connections. The thread will live until all connections have been established.",
              "lf_thread_create(&_fed.inbound_p2p_handling_thread_id,"
                  + " lf_handle_p2p_connections_from_federates, env);"));
    }

    for (FederateInstance remoteFederate : federate.outboundP2PConnections) {
      code.pr("lf_connect_to_federate(" + remoteFederate.id + ");");
    }
    return code.getCode();
  }

  /**
   * Generate code to handle physical actions in the `federate`.
   *
   * @param messageReporter Used to report errors.
   * @return Generated code.
   */
  private String generateCodeForPhysicalActions(
      FederateInstance federate, MessageReporter messageReporter) {
    CodeBuilder code = new CodeBuilder();
    var coordinationMode = federate.targetConfig.get(CoordinationProperty.INSTANCE);
    var coordinationOptions = federate.targetConfig.get(CoordinationOptionsProperty.INSTANCE);
    if (coordinationMode.equals(CoordinationMode.CENTRALIZED)) {
      // If this program uses centralized coordination then check
      // for outputs that depend on physical actions so that null messages can be
      // sent to the RTI.
      var main =
          new ReactorInstance(
              FedASTUtils.findFederatedReactor(federate.instantiation.eResource()),
              messageReporter,
              1);
      // Use the instantiation to create a new ReactorInstance so that it gets the parameters of the
      // original.
      var instance =
          new ReactorInstance(federate.instantiation, main, messageReporter, -1, List.of());
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
      if (federate.targetConfig.getOrDefault(DNETProperty.INSTANCE)) {
        ActionInstance found = federate.findPhysicalAction(instance);
        if (found != null) {
          String warning =
              String.join(
                  "\n",
                  "Found a physical action inside the federate "
                      + addDoubleQuotes(instance.getName()),
                  "and a signal downstream next event tag (DNET) will be used.",
                  "The signal DNET may increase the lag, the time difference between ",
                  "the time this physical action is scheduled and the time it is executed, ",
                  "specifically when this federate has multiple upstream reactors.",
                  "Consider disabling the signal DNET with a property {DNET: false}.");
          messageReporter.at(found.getDefinition()).warning(warning);
        }
      }
      if (minDelay != TimeValue.MAX_VALUE) {
        // Unless silenced, issue a warning.
        if (coordinationOptions.advanceMessageInterval == null) {
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
