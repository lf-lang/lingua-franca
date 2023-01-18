package org.lflang.federated.extensions;

import java.util.LinkedHashMap;

import org.lflang.Target;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.DockerGeneratorBase;
import org.lflang.generator.c.CDockerGenerator;

public class CCppExtension extends CExtension {
    @Override
    public Target getNetworkReactionTarget() {
        return Target.CCPP;
    }

}
