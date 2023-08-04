package org.lflang.federated.extensions;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.regex.Pattern;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.TargetConfig.ClockSyncOptions;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Expression;
import org.lflang.lf.Input;
import org.lflang.lf.ParameterReference;

public class CExtensionUtils {

  // Regular expression pattern for shared_ptr types.
  static final Pattern sharedPointerVariable =
      Pattern.compile("^(/\\*.*?\\*/)?std::shared_ptr<(?<type>((/\\*.*?\\*/)?(\\S+))+)>$");

  /**
   * Generate C code that initializes network actions.
   *
   * <p>These network actions will be triggered by federate.c whenever a message is received from
   * the network.
   *
   * @param federate The federate.
   * @param main The main reactor that contains the federate (used to lookup references).
   */
  public static String initializeTriggersForNetworkActions(
      FederateInstance federate, ReactorInstance main) {
    CodeBuilder code = new CodeBuilder();
    if (federate.networkMessageActions.size() > 0) {
      var actionTableCount = 0;
      var zeroDelayActionTableCount = 0;
      for (int i = 0; i < federate.networkMessageActions.size(); ++i) {
        // Find the corresponding ActionInstance.
        Action action = federate.networkMessageActions.get(i);
        var reactor = main.lookupReactorInstance(federate.networkReceiverInstantiations.get(i));
        var actionInstance = reactor.lookupActionInstance(action);
        var trigger = CUtil.actionRef(actionInstance, null);
        code.pr(
            "_lf_action_table["
                + (actionTableCount++)
                + "] = (lf_action_base_t*)&"
                + trigger
                + "; \\");
        if (federate.zeroDelayNetworkMessageActions.contains(action)) {
          code.pr(
              "_lf_zero_delay_action_table["
                  + (zeroDelayActionTableCount++)
                  + "] = (lf_action_base_t*)&"
                  + trigger
                  + "; \\");
        }
      }
    }
    return code.getCode();
  }

  /**
   * Generate C code that holds a sorted list of STP structs by time.
   *
   * <p>For decentralized execution, on every logical timestep, a thread will iterate through each
   * staa struct, wait for the designated offset time, and set the associated port status to absent
   * if it isn't known.
   *
   * @param federate The federate.
   */
  public static String stpStructs(FederateInstance federate) {
    CodeBuilder code = new CodeBuilder();
    federate.stpOffsets.sort((d1, d2) -> (int) (d1.time - d2.time));
    if (!federate.stpOffsets.isEmpty()) {
      // Create a static array of trigger_t pointers.
      // networkMessageActions is a list of Actions, but we
      // need a list of trigger struct names for ActionInstances.
      // There should be exactly one ActionInstance in the
      // main reactor for each Action.
      for (int i = 0; i < federate.stpOffsets.size(); ++i) {
        // Find the corresponding ActionInstance.
        List<Action> networkActions =
            federate.stpToNetworkActionMap.get(federate.stpOffsets.get(i));

        code.pr("staa_lst[" + i + "] = (staa_t*) malloc(sizeof(staa_t));");
        code.pr(
            "staa_lst["
                + i
                + "]->STAA = "
                + CTypes.getInstance().getTargetTimeExpr(federate.stpOffsets.get(i))
                + ";");
        code.pr("staa_lst[" + i + "]->numActions = " + networkActions.size() + ";");
        code.pr(
            "staa_lst["
                + i
                + "]->actions = (lf_action_base_t**) malloc(sizeof(lf_action_base_t*) * "
                + networkActions.size()
                + ");");
        var tableCount = 0;
        for (Action action : networkActions) {
          code.pr(
              "staa_lst["
                  + i
                  + "]->actions["
                  + (tableCount++)
                  + "] = _lf_action_table["
                  + federate.networkMessageActions.indexOf(action)
                  + "];");
        }
      }
    }
    return code.getCode();
  }

  /**
   * Create a port status field variable for a network input port "input" in the self struct of a
   * reactor.
   *
   * @param input The network input port
   * @return A string containing the appropriate variable
   */
  public static String createPortStatusFieldForInput(Input input) {
    StringBuilder builder = new StringBuilder();
    // If it is not a multiport, then we could re-use the port trigger, and nothing needs to be done
    if (ASTUtils.isMultiport(input)) {
      // If it is a multiport, then create an auxiliary list of port
      // triggers for each channel of
      // the multiport to keep track of the status of each channel
      // individually
      builder.append("trigger_t* _lf__").append(input.getName()).append("_network_port_status;\n");
    }
    return builder.toString();
  }

  /**
   * Given a connection 'delay' expression, return a string that represents the interval_t value of
   * the additional delay that needs to be applied to the outgoing message.
   *
   * <p>The returned additional delay in absence of after on network connection (i.e., if delay is
   * passed as a null) is NEVER. This has a special meaning in C library functions that send network
   * messages that carry timestamps (@see send_timed_message and send_port_absent_to_federate in
   * lib/core/federate.c). In this case, the sender will send its current tag as the timestamp of
   * the outgoing message without adding a microstep delay. If the user has assigned an after delay
   * to the network connection (that can be zero) either as a time value (e.g., 200 msec) or as a
   * literal (e.g., a parameter), that delay in nsec will be returned.
   *
   * @param delay The delay associated with a connection.
   */
  public static String getNetworkDelayLiteral(Expression delay) {
    var d = ASTUtils.getDelay(delay);
    return d == null ? "NEVER" : Long.toString(d);
  }

  static boolean isSharedPtrType(InferredType type, CTypes types) {
    return !type.isUndefined() && sharedPointerVariable.matcher(types.getTargetType(type)).find();
  }

  public static void handleCompileDefinitions(
      FederateInstance federate,
      int numOfFederates,
      RtiConfig rtiConfig,
      MessageReporter messageReporter) {
    federate.targetConfig.setByUser.add(TargetProperty.COMPILE_DEFINITIONS);
    federate.targetConfig.compileDefinitions.put("FEDERATED", "");
    federate.targetConfig.compileDefinitions.put(
        "FEDERATED_" + federate.targetConfig.coordination.toString().toUpperCase(), "");
    if (federate.targetConfig.auth) {
      federate.targetConfig.compileDefinitions.put("FEDERATED_AUTHENTICATED", "");
    }
    federate.targetConfig.compileDefinitions.put(
        "NUMBER_OF_FEDERATES", String.valueOf(numOfFederates));
    federate.targetConfig.compileDefinitions.put("EXECUTABLE_PREAMBLE", "");

    handleAdvanceMessageInterval(federate);

    initializeClockSynchronization(federate, rtiConfig, messageReporter);
  }

  private static void handleAdvanceMessageInterval(FederateInstance federate) {
    var advanceMessageInterval = federate.targetConfig.coordinationOptions.advance_message_interval;
    federate.targetConfig.setByUser.remove(TargetProperty.COORDINATION_OPTIONS);
    if (advanceMessageInterval != null) {
      federate.targetConfig.compileDefinitions.put(
          "ADVANCE_MESSAGE_INTERVAL", String.valueOf(advanceMessageInterval.toNanoSeconds()));
    }
  }

  static boolean clockSyncIsOn(FederateInstance federate, RtiConfig rtiConfig) {
    return federate.targetConfig.clockSync != ClockSyncMode.OFF
        && (!rtiConfig.getHost().equals(federate.host)
            || federate.targetConfig.clockSyncOptions.localFederatesOn);
  }

  /**
   * Initialize clock synchronization (if enabled) and its related options for a given federate.
   *
   * <p>Clock synchronization can be enabled using the clock-sync target property.
   *
   * @see <a
   *     href="https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization">Documentation</a>
   */
  public static void initializeClockSynchronization(
      FederateInstance federate, RtiConfig rtiConfig, MessageReporter messageReporter) {
    // Check if clock synchronization should be enabled for this federate in the first place
    if (clockSyncIsOn(federate, rtiConfig)) {
      messageReporter
          .nowhere()
          .info("Initial clock synchronization is enabled for federate " + federate.id);
      if (federate.targetConfig.clockSync == ClockSyncMode.ON) {
        if (federate.targetConfig.clockSyncOptions.collectStats) {
          messageReporter
              .nowhere()
              .info("Will collect clock sync statistics for federate " + federate.id);
          // Add libm to the compiler flags
          // FIXME: This is a linker flag not compile flag but we don't have a way to add linker
          // flags
          // FIXME: This is probably going to fail on MacOS (especially using clang)
          // because libm functions are builtin
          federate.targetConfig.compilerFlags.add("-lm");
          federate.targetConfig.setByUser.add(TargetProperty.FLAGS);
        }
        messageReporter
            .nowhere()
            .info("Runtime clock synchronization is enabled for federate " + federate.id);
      }

      addClockSyncCompileDefinitions(federate);
    }
  }

  /**
   * Initialize clock synchronization (if enabled) and its related options for a given federate.
   *
   * <p>Clock synchronization can be enabled using the clock-sync target property.
   *
   * @see <a
   *     href="https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization">Documentation</a>
   */
  public static void addClockSyncCompileDefinitions(FederateInstance federate) {

    ClockSyncMode mode = federate.targetConfig.clockSync;
    ClockSyncOptions options = federate.targetConfig.clockSyncOptions;

    federate.targetConfig.compileDefinitions.put("_LF_CLOCK_SYNC_INITIAL", "");
    federate.targetConfig.compileDefinitions.put(
        "_LF_CLOCK_SYNC_PERIOD_NS", String.valueOf(options.period.toNanoSeconds()));
    federate.targetConfig.compileDefinitions.put(
        "_LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL", String.valueOf(options.trials));
    federate.targetConfig.compileDefinitions.put(
        "_LF_CLOCK_SYNC_ATTENUATION", String.valueOf(options.attenuation));

    if (mode == ClockSyncMode.ON) {
      federate.targetConfig.compileDefinitions.put("_LF_CLOCK_SYNC_ON", "");
      if (options.collectStats) {
        federate.targetConfig.compileDefinitions.put("_LF_CLOCK_SYNC_COLLECT_STATS", "");
      }
    }
  }

  /** Generate a file to be included by CMake. */
  public static void generateCMakeInclude(FederateInstance federate, FedFileConfig fileConfig)
      throws IOException {
    Files.createDirectories(fileConfig.getSrcPath().resolve("include"));

    Path cmakeIncludePath =
        fileConfig
            .getSrcPath()
            .resolve("include" + File.separator + federate.name + "_extension.cmake");

    CodeBuilder cmakeIncludeCode = new CodeBuilder();

    cmakeIncludeCode.pr(generateSerializationCMakeExtension(federate));
    cmakeIncludeCode.pr(
        "add_compile_definitions(LF_SOURCE_DIRECTORY=\"" + fileConfig.srcPath + "\")");
    cmakeIncludeCode.pr(
        "add_compile_definitions(LF_PACKAGE_DIRECTORY=\"" + fileConfig.srcPkgPath + "\")");

    try (var srcWriter = Files.newBufferedWriter(cmakeIncludePath)) {
      srcWriter.write(cmakeIncludeCode.getCode());
    }

    federate.targetConfig.cmakeIncludes.add(
        fileConfig.getSrcPath().relativize(cmakeIncludePath).toString());
    federate.targetConfig.setByUser.add(TargetProperty.CMAKE_INCLUDE);
  }

  /**
   * Generate code that sends the neighbor structure message to the RTI. See {@code
   * MSG_TYPE_NEIGHBOR_STRUCTURE} in {@code federated/net_common.h}.
   *
   * @param federate The federate that is sending its neighbor structure
   */
  public static String generateFederateNeighborStructure(FederateInstance federate) {
    var code = new CodeBuilder();
    code.pr(
        String.join(
            "\n",
            "/**",
            "* Generated function that sends information about connections between this federate"
                + " and",
            "* other federates where messages are routed through the RTI. Currently, this",
            "* only includes logical connections when the coordination is centralized. This",
            "* information is needed for the RTI to perform the centralized coordination.",
            "* @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h",
            "*/",
            "void send_neighbor_structure_to_RTI(int rti_socket) {"));
    code.indent();
    // Initialize the array of information about the federate's immediate upstream
    // and downstream relayed (through the RTI) logical connections, to send to the
    // RTI.
    code.pr(
        String.join(
            "\n",
            "interval_t candidate_tmp;",
            "size_t buffer_size = 1 + 8 + ",
            "                "
                + federate.dependsOn.keySet().size()
                + " * ( sizeof(uint16_t) + sizeof(int64_t) ) +",
            "                " + federate.sendsTo.keySet().size() + " * sizeof(uint16_t);",
            "unsigned char buffer_to_send[buffer_size];",
            "",
            "size_t message_head = 0;",
            "buffer_to_send[message_head] = MSG_TYPE_NEIGHBOR_STRUCTURE;",
            "message_head++;",
            "encode_int32((int32_t)"
                + federate.dependsOn.keySet().size()
                + ", &(buffer_to_send[message_head]));",
            "message_head+=sizeof(int32_t);",
            "encode_int32((int32_t)"
                + federate.sendsTo.keySet().size()
                + ", &(buffer_to_send[message_head]));",
            "message_head+=sizeof(int32_t);"));

    if (!federate.dependsOn.keySet().isEmpty()) {
      // Next, populate these arrays.
      // Find the minimum delay in the process.
      // FIXME: Zero delay is not really the same as a microstep delay.
      for (FederateInstance upstreamFederate : federate.dependsOn.keySet()) {
        code.pr(
            String.join(
                "\n",
                "encode_uint16((uint16_t)"
                    + upstreamFederate.id
                    + ", &(buffer_to_send[message_head]));",
                "message_head += sizeof(uint16_t);"));
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
          for (Expression delay : delays) {
            if (delay == null) {
              // Use NEVER to encode no delay at all.
              code.pr("candidate_tmp = NEVER;");
            } else {
              var delayTime =
                  delay instanceof ParameterReference
                      // In that case use the default value.
                      ? CTypes.getInstance()
                          .getTargetTimeExpr(
                              ASTUtils.getDefaultAsTimeValue(
                                  ((ParameterReference) delay).getParameter()))
                      : CTypes.getInstance().getTargetExpr(delay, InferredType.time());

              code.pr(
                  String.join(
                      "\n",
                      "if (" + delayTime + " < candidate_tmp) {",
                      "    candidate_tmp = " + delayTime + ";",
                      "}"));
            }
          }
          code.pr(
              String.join(
                  "\n",
                  "encode_int64((int64_t)candidate_tmp, &(buffer_to_send[message_head]));",
                  "message_head += sizeof(int64_t);"));
        } else {
          // Use NEVER to encode no delay at all.
          code.pr(
              String.join(
                  "\n",
                  "encode_int64(NEVER, &(buffer_to_send[message_head]));",
                  "message_head += sizeof(int64_t);"));
        }
      }
    }

    // Next, set up the downstream array.
    if (!federate.sendsTo.keySet().isEmpty()) {
      // Next, populate the array.
      // Find the minimum delay in the process.
      // FIXME: Zero delay is not really the same as a microstep delay.
      for (FederateInstance downstreamFederate : federate.sendsTo.keySet()) {
        code.pr(
            String.join(
                "\n",
                "encode_uint16(" + downstreamFederate.id + ", &(buffer_to_send[message_head]));",
                "message_head += sizeof(uint16_t);"));
      }
    }
    code.pr(
        String.join(
            "\n",
            "write_to_socket_errexit(",
            "    rti_socket, ",
            "    buffer_size,",
            "    buffer_to_send,",
            "    \"Failed to send the neighbor structure message to the RTI.\"",
            ");"));
    code.unindent();
    code.pr("}");
    return code.toString();
  }

  /**
   * Surround {@code code} with blocks to ensure that code only executes if the program is
   * federated.
   */
  public static String surroundWithIfFederated(String code) {
    return """
            #ifdef FEDERATED
            %s
            #endif // FEDERATED
            """
        .formatted(code);
  }

  /**
   * Surround {@code code} with blocks to ensure that code only executes if the program is federated
   * and has a centralized coordination.
   */
  public static String surroundWithIfFederatedCentralized(String code) {
    return """
            #ifdef FEDERATED_CENTRALIZED
            %s
            #endif // FEDERATED_CENTRALIZED
            """
        .formatted(code);
  }

  /**
   * Surround {@code code} with blocks to ensure that code only executes if the program is federated
   * and has a decentralized coordination.
   */
  public static String surroundWithIfFederatedDecentralized(String code) {
    return """
            #ifdef FEDERATED_DECENTRALIZED
            %s
            #endif // FEDERATED_DECENTRALIZED
            """
        .formatted(code);
  }

  /** Generate preamble code needed for enabled serializers of the federate. */
  public static String generateSerializationIncludes(FederateInstance federate) {
    CodeBuilder code = new CodeBuilder();
    for (SupportedSerializers serializer : federate.enabledSerializers) {
      switch (serializer) {
        case NATIVE, PROTO -> {
          // No need to do anything at this point.
        }
        case ROS2 -> {
          var ROSSerializer = new FedROS2CPPSerialization();
          code.pr(ROSSerializer.generatePreambleForSupport().toString());
        }
      }
    }
    return code.getCode();
  }

  /** Generate cmake-include code needed for enabled serializers of the federate. */
  public static String generateSerializationCMakeExtension(FederateInstance federate) {
    CodeBuilder code = new CodeBuilder();
    for (SupportedSerializers serializer : federate.enabledSerializers) {
      switch (serializer) {
        case NATIVE, PROTO -> {
          // No CMake code is needed for now
        }
        case ROS2 -> {
          var ROSSerializer = new FedROS2CPPSerialization();
          code.pr(ROSSerializer.generateCompilerExtensionForSupport());
        }
      }
    }
    return code.getCode();
  }
}
