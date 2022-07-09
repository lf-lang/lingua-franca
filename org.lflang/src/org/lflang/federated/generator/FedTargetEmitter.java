package org.lflang.federated.generator;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.ast.ToLf;
import org.lflang.federated.extensions.FedTargetExtension;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.GeneratorUtils;
import org.lflang.lf.KeyValuePair;

public class FedTargetEmitter {

    String generateTarget(
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter,
        LinkedHashMap<String, Object> federationRTIProperties
    ) throws IOException {
        FedTargetExtensionFactory.getExtension(federate.target)
                                 .initializeTargetConfig(federate, fileConfig, errorReporter, federationRTIProperties);

        // FIXME: handle existing cmake-includes
        return ToLf.instance.doSwitch(federate.target);
    }
}