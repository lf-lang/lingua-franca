package org.lflang.federated.extensions;

import org.lflang.Target;
import org.lflang.lf.TargetDecl;

/**
 * FIXME
 */
public class FedTargetExtensionFactory {

    public static FedTargetExtension getExtension(TargetDecl target) {
        switch (Target.fromDecl(target)) {
        case C: return new CExtension();
        case CCPP: return new CCppExtension();
        case Python: return new PythonExtension();
        case TS: return new TSExtension();
        default: throw new RuntimeException("Target not supported");
        }
    }
}
