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
        switch (target) {
            case CCPP:
            case C:
                return new CExtension();
            case Python:
                return new PythonExtension();
            case TS:
                return new TSExtension();
            default:
                throw new RuntimeException("Target not supported");
        }
    }
}
