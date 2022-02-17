package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import org.lflang.TargetProperty.CoordinationType;

public class CPreambleGenerator {
    /**
     * Returns the #define directive for the given coordination type.
     * 
     * NOTE: Instead of checking #ifdef FEDERATED, we could
     *       use #if (NUMBER_OF_FEDERATES > 1).
     *       To Soroush Bateni, the former is more accurate.
     */
    public static String generateFederatedDirective(CoordinationType coordinationType) {        
        List<String> directives = new ArrayList<>();
        directives.add("#define FEDERATED");
        if (coordinationType == CoordinationType.CENTRALIZED) {
            directives.add("#define FEDERATED_CENTRALIZED");
        } else if (coordinationType == CoordinationType.DECENTRALIZED) {
            directives.add("#define FEDERATED_DECENTRALIZED");
        }
        return String.join("\n", directives);
    }
}
