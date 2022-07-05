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

import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.xbase.lib.IterableExtensions;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.serialization.FedNativePythonSerialization;
import org.lflang.federated.serialization.FedSerialization;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

/**
 * An extension class to the PythonGenerator that enables certain federated
 * functionalities.
 *
 * @author Soroush Bateni {soroush@utdallas.edu}
 */
public class PythonExtension extends CExtension {

    /**
     * FIXME
     *
     * @param action
     * @param receivingPort
     * @param connection
     * @param type
     * @param receiveRef
     * @param result
     * @param errorReporter
     */
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
            throw new UnsupportedOperationException("Protbuf serialization is not supported yet.");
        }
        case ROS2: {
            throw new UnsupportedOperationException("ROS2 serialization is not supported yet.");
        }

        }
    }

    /**
     * FIXME
     *
     * @param connection
     * @param type
     * @param sendRef
     * @param result
     * @param sendingFunction
     * @param commonArgs
     * @param errorReporter
     */
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
    public String generatePreamble(FederateInstance federate) {
//        if (!IterableExtensions.isNullOrEmpty(targetConfig.protoFiles)) {
//            // Enable support for proto serialization
//            enabledSerializers.add(SupportedSerializers.PROTO);
//        }
//        for (SupportedSerializers serialization : enabledSerializers) {
//            switch (serialization) {
//            case NATIVE: {
//                FedNativePythonSerialization pickler = new FedNativePythonSerialization();
//                code.pr(pickler.generatePreambleForSupport().toString());
//            }
//            case PROTO: {
//                // Handle .proto files.
//                for (String name : targetConfig.protoFiles) {
//                    this.processProtoFile(name, cancelIndicator);
//                    int dotIndex = name.lastIndexOf(".");
//                    String rootFilename = dotIndex > 0 ? name.substring(0, dotIndex) : name;
//                    pythonPreamble.pr("import "+rootFilename+"_pb2 as "+rootFilename);
//                    protoNames.add(rootFilename);
//                }
//            }
//            case ROS2: {
//                // FIXME: Not supported yet
//            }
//            }
        return null;
        }
    }
