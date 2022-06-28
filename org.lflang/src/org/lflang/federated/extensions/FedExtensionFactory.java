package org.lflang.federated.extensions;

import org.lflang.Target;
import org.lflang.lf.TargetDecl;

/**
 * FIXME
 */
public class FedExtensionFactory {

    public static FedGeneratorExtension getExtension(TargetDecl target) {
        switch (Target.fromDecl(target)) {
        case C: return new CGeneratorExtension();
        case Python: return new PythonGeneratorExtension();
        default: throw new RuntimeException("Target not supported");
        }
    }
}
