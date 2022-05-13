package org.lflang.generator.python;

import org.lflang.lf.Action;
import org.lflang.lf.ReactorDecl;
import org.lflang.generator.c.CGenerator;

public class PythonActionGenerator {
    public static String generateAliasTypeDef(ReactorDecl decl, Action action,
                                              String genericActionType) {

        return "typedef "+genericActionType+" "+CGenerator.variableStructType(action, decl)+";";
    }
}
