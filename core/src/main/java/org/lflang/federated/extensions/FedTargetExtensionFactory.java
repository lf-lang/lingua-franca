package org.lflang.federated.extensions;

import org.lflang.Target;

/**
 * Class for instantiating target extensions.
 *
 * @author Soroush Bateni
 */
public class FedTargetExtensionFactory {

  /** Given a target, return the appropriate extension. */
  public static FedTargetExtension getExtension(Target target) {
    return switch (target) {
      case CCPP, C -> new CExtension();
      case Python -> new PythonExtension();
      case TS -> new TSExtension();
      default -> throw new RuntimeException("Target not supported");
    };
  }
}
