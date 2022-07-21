package org.lflang.federated.extensions;

import java.io.IOException;
import java.nio.file.Files;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.FedTSLauncher;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

public class TSExtension implements FedTargetExtension {

    @Override
    public void initializeTargetConfig(LFGeneratorContext context, FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException {

    }

    @Override
    public void createLauncher(List<FederateInstance> federates, FileConfig fileConfig, TargetConfig targetConfig, ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException {
        // Create bin directory for the script.
        if (!Files.exists(fileConfig.binPath)) {
            Files.createDirectories(fileConfig.binPath);
        }
        // Generate script for launching federation
        var launcher = new FedTSLauncher(targetConfig, fileConfig, errorReporter);
        launcher.createLauncher(federates, federationRTIProperties);
        // TODO(hokeun): Modify this to make this work with standalone RTI.
        // If this is a federated execution, generate C code for the RTI.
//            // Copy the required library files into the target file system.
//            // This will overwrite previous versions.
//            var files = ArrayList("rti.c", "rti.h", "federate.c", "reactor_threaded.c", "reactor.c", "reactor_common.c", "reactor.h", "pqueue.c", "pqueue.h", "util.h", "util.c")
//
//            for (file : files) {
//                copyFileFromClassPath(
//                    File.separator + "lib" + File.separator + "core" + File.separator + file,
//                    fileConfig.getSrcGenPath.toString + File.separator + file
//                )
//            }
    }

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {

        return """
        // generateNetworkReceiverBody
        if (%1$s !== undefined) {
            %s.%s = %1$s;
        }
        """.formatted(
            action.getName(),
            receivingPort.getContainer().getName(),
            receivingPort.getVariable().getName()
        );
    }

    @Override
    public String generateNetworkSenderBody(VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return"""
        if (%1$s.%2$s !== undefined) {
            this.util.sendRTITimedMessage(%1$s.%2$s, %s, %s);
        }
        """.formatted(
            sendingPort.getContainer().getName(),
            sendingPort.getVariable().getName(),
            connection.getDstFederate().id,
            connection.getDstFederate().networkMessageActions.size()
        );
    }

    @Override
    public String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP, CoordinationType coordination) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkInputControlReactionBody";
    }

    @Override
    public String generateNetworkOutputControlReactionBody(VarRef srcOutputPort, FedConnectionInstance connection) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkOutputControlReactionBody";
    }

    @Override
    public String getNetworkBufferType() {
        return "";
    }

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @return
     */
    @Override
    public String generatePreamble(FederateInstance federate, LinkedHashMap<String, Object> federationRTIProperties, Integer numOfFederates, ErrorReporter errorReporter) {
//        for (serializer in enabledSerializers) {
//            when (serializer) {
//                SupportedSerializers.NATIVE -> {
//                    // No need to do anything at this point.
//                    println("Native serializer is enabled.")
//                }
//                else -> throw UnsupportedOperationException("Unsupported serializer: $serializer");
//            }
//        }
        return
        """
        preamble {=
            %s
        =}""".formatted("");
    }
}
