package org.lflang.federated.extensions;

import org.lflang.Target;

/**
 * FIXME
 */
public class FedExtensionFactory {

    public static FedGeneratorExtension getExtension(Target target) {
        switch (target) {
        case C: return new CGeneratorExtension;
        case Python: return new PythonGeneratorExtension;
        default: throw new RuntimeException("Target not supported");
        }
    }
}
