package org.lflang.federated.generator;

import java.util.stream.Collectors;

import org.lflang.ast.FormattingUtils;
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
            .map(FormattingUtils.renderer(federate.target))
            .collect(Collectors.joining("\n"));
    }
}
