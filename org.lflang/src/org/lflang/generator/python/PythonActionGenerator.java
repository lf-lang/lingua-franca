package org.lflang.generator.python;

import org.lflang.generator.c.CGenerator;
import org.lflang.lf.Action;
import org.lflang.lf.Reactor;

public class PythonActionGenerator {
    public static String generateAliasTypeDef(Reactor r, Action action, String genericActionType) {

        return "typedef " + genericActionType + " " + CGenerator.variableStructType(action, r, false) + ";";
    }
}
