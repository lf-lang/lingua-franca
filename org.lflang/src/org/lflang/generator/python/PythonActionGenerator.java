package org.lflang.generator.python;

import org.lflang.generator.c.TypeParameterizedReactor;
import org.lflang.lf.Action;
import org.lflang.lf.Reactor;
import org.lflang.generator.c.CGenerator;

public class PythonActionGenerator {
    public static String generateAliasTypeDef(TypeParameterizedReactor tpr, Action action,
                                              String genericActionType) {

        return "typedef "+genericActionType+" "+CGenerator.variableStructType(action, tpr, false)+";";
    }
}
