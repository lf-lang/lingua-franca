package org.lflang.federated.extensions;

import java.io.IOException;
import java.util.List;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Action;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;
import org.lflang.target.property.type.CoordinationModeType.CoordinationMode;

/**
 * Interface for federated target extensions.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public interface FedTargetExtension {

  /**
   * Perform necessary actions to initialize the target config.
   *
   * @param context The context of the original code generation process.
   * @param federateNames The names of all the federates in the program.
   * @param federate The federate instance.
   * @param fileConfig An instance of `FedFileConfig`.
   * @param messageReporter Used to report errors.
   * @param rtiConfig The RTI configuration settings.
   */
  void initializeTargetConfig(
      LFGeneratorContext context,
      List<String> federateNames,
      FederateInstance federate,
      FederationFileConfig fileConfig,
      MessageReporter messageReporter,
      RtiConfig rtiConfig)
      throws IOException;

  /**
   * Generate code for the body of a reaction that handles the action that is triggered by receiving
   * a message from a remote federate.
   *
   * @param action The action.
   * @param sendingPort The output port providing the data to send.
   * @param receivingPort The ID of the destination port.
   * @param connection The federated connection being lowered.
   * @param type The type of the data being sent over the connection.
   * @param coordinationMode The coordination type
   * @param messageReporter Used to report errors and warnings.
   */
  String generateNetworkReceiverBody(
      Action action,
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
      MessageReporter messageReporter);

  /** Generate code for initializing a network output reactor from its startup reaction. */
  String outputInitializationBody();

  /** Generate code for the parameter that specifies the sender index. */
  void addSenderIndexParameter(Reactor sender);

  /** Generate code for the sender index argument of `instantiation`. */
  void supplySenderIndexParameter(Instantiation inst, int idx);

  /**
   * Generate code for the body of a reaction that handles an output that is to be sent over the
   * network.
   *
   * @param sendingPort The output port providing the data to send.
   * @param receivingPort The variable reference to the destination port.
   * @param connection The federated connection being lowered.
   * @param type The type of the data being sent over the connection.
   * @param coordinationMode Whether the federated program is centralized or decentralized.
   * @param messageReporter Used to report errors and warnings.
   */
  String generateNetworkSenderBody(
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
      MessageReporter messageReporter);

  /**
   * Generate code for the body of a reaction that sends a port status message for the given port if
   * it is absent.
   *
   * @param srcOutputPort A reference to the output port of the federate instance.
   * @param connection The federated connection being lowered.
   */
  String generatePortAbsentReactionBody(VarRef srcOutputPort, FedConnectionInstance connection);

  /** Optionally apply additional annotations to the reaction. */
  default void annotateReaction(Reaction reaction) {}

  /**
   * Return the type for the raw network buffer in the target language (e.g., `uint_8` in C).
   * This would be the type of the network messages after serialization and before deserialization.
   * It is primarily used to determine the type for the network action at the receiver.
   */
  String getNetworkBufferType();

  /**
   * Add preamble to the source to set up federated execution.
   *
   * @param federate The federate to which the generated setup code will correspond.
   * @param fileConfig The federation file configuration.
   * @param rtiConfig The settings of the RTI.
   * @param messageReporter Used to report errors and warnings.
   */
  String generatePreamble(
      FederateInstance federate,
      FederationFileConfig fileConfig,
      RtiConfig rtiConfig,
      MessageReporter messageReporter)
      throws IOException;
}
