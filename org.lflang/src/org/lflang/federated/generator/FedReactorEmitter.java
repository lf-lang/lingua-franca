package org.lflang.federated.generator;

import java.util.stream.Collectors;

import org.lflang.ast.ToLf;
import org.lflang.lf.Model;

public class FedReactorEmitter {

    public FedReactorEmitter() {}

    /**
     * @param federate
     * @return
     */
    String generateReactorDefinitions(FederateInstance federate) {
        return ((Model) federate.instantiation.eContainer().eContainer())
            .getReactors()
            .stream()
            .filter(federate::contains)
            .map(ToLf.instance::doSwitch)
            .collect(Collectors.joining("\n"));
    }
}