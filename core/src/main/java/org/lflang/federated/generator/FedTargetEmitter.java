package org.lflang.federated.generator;

import java.io.IOException;
import org.lflang.MessageReporter;
import org.lflang.TargetProperty;
import org.lflang.ast.FormattingUtil;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.LFGeneratorContext;

public class FedTargetEmitter {

  String generateTarget(
      LFGeneratorContext context,
      int numOfFederates,
      FederateInstance federate,
      FedFileConfig fileConfig,
      MessageReporter messageReporter,
      RtiConfig rtiConfig)
      throws IOException {

    // FIXME: First of all, this is not an initialization; there is all sorts of stuff happening
    // in the C implementation of this method. Second, true initialization stuff should happen
    // when the target config is constructed, not when we're doing code generation.
    // See https://issues.lf-lang.org/1667
    FedTargetExtensionFactory.getExtension(federate.targetConfig.target)
        .initializeTargetConfig(
            context, numOfFederates, federate, fileConfig, messageReporter, rtiConfig);

    return FormattingUtil.renderer(federate.targetConfig.target)
        .apply(
            TargetProperty.extractTargetDecl(federate.targetConfig.target, federate.targetConfig));
  }
}
