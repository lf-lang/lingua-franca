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

package org.lflang.federated.extensions;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.launcher.FedPyLauncher;
import org.lflang.federated.serialization.FedNativePythonSerialization;
import org.lflang.federated.serialization.FedSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.python.PyUtil;
import org.lflang.generator.python.PythonDockerGenerator;
import org.lflang.generator.python.PythonInfoGenerator;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

/**
 * An extension class to the PythonGenerator that enables certain federated
 * functionalities.
 *
 * @author Soroush Bateni {soroush@utdallas.edu}
 */
public class PythonExtension extends CExtension {

    @Override
    protected void generateCMakeInclude(FederateInstance federate, FedFileConfig fileConfig) throws IOException {}

    @Override
    protected String generateSerializationPreamble(FederateInstance federate, FedFileConfig fileConfig) {
        CodeBuilder code = new CodeBuilder();
        for (SupportedSerializers serialization : federate.enabledSerializers) {
            switch (serialization) {
            case NATIVE: {
                FedNativePythonSerialization pickler = new FedNativePythonSerialization();
                code.pr(pickler.generatePreambleForSupport().toString());
            }
            case PROTO: {
                // Nothing needs to be done
            }
            case ROS2: {
                // FIXME: Not supported yet
            }
            }
        }
        return code.getCode();
    }

    @Override
    public String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    ) {
        var result = new CodeBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        result.pr(PyUtil.generateGILAcquireCode() + "\n");
        result.pr(
            super.generateNetworkSenderBody(
                sendingPort,
                receivingPort,
                connection,
                type,
                coordinationType,
                errorReporter
            )
        );
        result.pr(PyUtil.generateGILReleaseCode() + "\n");
        return result.getCode();
    }

    @Override
    public String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    ) {
        var result = new CodeBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.pr("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");
        result.pr(PyUtil.generateGILAcquireCode() + "\n");
        result.pr(
            super.generateNetworkReceiverBody(
                action,
                sendingPort,
                receivingPort,
                connection,
                type,
                coordinationType,
                errorReporter
            )
        );
        result.pr(PyUtil.generateGILReleaseCode() + "\n");
        return result.getCode();

    }

    @Override
    protected void deserialize(
        Action action,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        String receiveRef,
        CodeBuilder result,
        ErrorReporter errorReporter
    ) {
        String value = "";
        switch (connection.getSerializer()) {
        case NATIVE: {
            value = action.getName();
            FedNativePythonSerialization pickler = new FedNativePythonSerialization();
            result.pr(pickler.generateNetworkDeserializerCode(value, null));
            result.pr("lf_set(" + receiveRef + ", "
                          + FedSerialization.deserializedVarName + ");\n");
            break;
        }
        case PROTO: {
            throw new UnsupportedOperationException("Protobuf serialization is not supported yet.");
        }
        case ROS2: {
            throw new UnsupportedOperationException("ROS2 serialization is not supported yet.");
        }

        }
    }

    @Override
    protected void serializeAndSend(
        FedConnectionInstance connection,
        InferredType type,
        String sendRef,
        CodeBuilder result,
        String sendingFunction,
        String commonArgs,
        ErrorReporter errorReporter
    ) {
        String lengthExpression = "";
        String pointerExpression = "";
        switch (connection.getSerializer()) {
        case NATIVE: {
            var variableToSerialize = sendRef + "->value";
            FedNativePythonSerialization pickler = new FedNativePythonSerialization();
            lengthExpression = pickler.serializedBufferLength();
            pointerExpression = pickler.seializedBufferVar();
            result.pr(pickler.generateNetworkSerializerCode(variableToSerialize, null));
            result.pr(
                "size_t message_length = " + lengthExpression + ";");
            result.pr(
                sendingFunction + "(" + commonArgs + ", " + pointerExpression
                    + ");\n");
            break;
        }
        case PROTO: {
            throw new UnsupportedOperationException("Protbuf serialization is not supported yet.");
        }
        case ROS2: {
            throw new UnsupportedOperationException("ROS2 serialization is not supported yet.");
        }

        }
    }

    @Override
    protected DockerGeneratorBase newDockerGeneratorInstance(FederateInstance federate) {
        return new PythonDockerGenerator(true, federate.targetConfig);
    }

    @Override
    public Target getNetworkReactionTarget() {
        return Target.C;
    }
}
