package org.lflang.generator.c;

import java.util.List;
import java.util.ArrayList;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactorInstance;


public class CActionGenerator {
    /**
     * For each action of the specified reactor instance, generate initialization code
     * for the offset and period fields. 
     * @param instance The reactor.
     * @param currentFederate The federate we are 
     */
    public static String generateInitializers(
        ReactorInstance instance, 
        FederateInstance currentFederate
    ) {
        List<String> code = new ArrayList<>();
        for (ActionInstance action : instance.actions) {
            if (currentFederate.contains(action.getDefinition()) &&
                !action.isShutdown()
            ) {
                var triggerStructName = CUtil.reactorRef(action.getParent()) + "->_lf__" + action.getName();
                var minDelay = action.getMinDelay();
                var minSpacing = action.getMinSpacing();
                var offsetInitializer = triggerStructName+".offset = "+GeneratorBase.timeInTargetLanguage(minDelay);
                var periodInitializer = minSpacing == null ? 
                                        triggerStructName+".period = "+GeneratorBase.timeInTargetLanguage(minDelay) :
                                        triggerStructName+".period = "+CGenerator.UNDEFINED_MIN_SPACING;
                code.addAll(List.of(
                    "// Initializing action "+action.getFullName(),
                    offsetInitializer,
                    periodInitializer
                ));
                
                var mode = action.getMode(false);
                if (mode != null) {
                    var modeParent = mode.getParent();
                    var modeRef = "&"+CUtil.reactorRef(modeParent)+"->_lf__modes["+modeParent.modes.indexOf(mode)+"];";
                    code.add(triggerStructName+".mode = "+modeRef+";");
                } else {
                    code.add(triggerStructName+".mode = NULL;");
                }
            }
        }
        return String.join("\n", code);
    }
}
