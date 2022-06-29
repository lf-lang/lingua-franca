package org.lflang.federated.generator;

import org.lflang.ast.ToLf;

public class FedTargetEmitter {

    String generateTarget(FederateInstance federate) {
        return ToLf.instance.doSwitch(federate.target);
    }
}