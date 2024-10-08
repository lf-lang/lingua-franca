package org.lflang.analyses.uclid;

import java.util.List;

import org.lflang.ast.ASTUtils;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.target.TargetConfig;

public class CbmcGenerator {

    public void doGenerate(TargetConfig targetConfig, Instantiation mainDef) {
        List<Reactor> reactorDefs = ASTUtils.getAllReactors(targetConfig.getMainResource());
        for (Reactor reactorDef : reactorDefs) {
            List<Reaction> reactionDefs = reactorDef.getReactions();
            for (Reaction reactionDef : reactionDefs) {
                System.out.println("Reaction " + reactionDef.getName());
                System.out.println(reactionDef.getCode().getBody());
            }
        }
    }
    
}
