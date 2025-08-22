package org.lflang.federated.generator;

import java.util.stream.Collectors;
import org.lflang.ast.FormattingUtil;
import org.lflang.lf.Model;

/**
 * Helper class to generate reactor definitions for a federate.
 *
 * @ingroup Federated
 */
public class FedReactorEmitter {

  public FedReactorEmitter() {}

  /** Return textual representations of all reactor classes belonging to {@code federate}. */
  String generateReactorDefinitions(FederateInstance federate) {
    return ((Model) federate.instantiation.eContainer().eContainer())
        .getReactors().stream()
            .distinct()
            .filter(federate::references)
            .map(FormattingUtil.renderer(federate.targetConfig.target))
            .collect(Collectors.joining("\n"));
  }
}
