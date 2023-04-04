package org.lflang.federated.extensions;

import org.lflang.Target;
import org.lflang.lf.TargetDecl;

/**
 * Class for instantiating target extensions.
 * @author Soroush Bateni
 */
public class FedTargetExtensionFactory {

    /**
     * Given a target, return the appropriate extension.
     */
    public static FedTargetExtension getExtension(TargetDecl target) {
        switch (Target.fromDecl(target)) {
        case CCPP:
        case C: return new CExtension();
        case Python: return new PythonExtension();
        case TS: return new TSExtension();
        default: throw new RuntimeException("Target not supported");
        }
    }
}
