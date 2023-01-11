package org.lflang.generator.c;

import static org.lflang.ASTUtils.getInferredType;

import org.eclipse.core.internal.resources.Resource;

import org.lflang.ASTUtils;
import org.lflang.Target;
import org.lflang.generator.DelayGenerator;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

public class CDelayGenerator extends DelayGenerator {

    protected CTypes types;

    public CDelayGenerator(Resource resource, CTypes types) {
        super(resource, Target.C, false, false);
        this.types = types;
    }

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port. This realizes
     * the receiving end of a logical delay specified with the 'after'
     * keyword.
     * @param action The action that triggers the reaction
     * @param port The port to write to.
     */
    @Override
    public String generateForwardBody(Action action, VarRef port) {
        var outputName = ASTUtils.generateVarRef(port);
        return CReactionGenerator.generateForwardBody(
            outputName,
            types.getTargetType(action),
            action.getName(),
            CUtil.isTokenType(getInferredType(action), types)
        );
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     * @param action The action to schedule
     * @param port The port to read from
     */
    @Override
    public String generateDelayBody(Action action, VarRef port) {
        var ref = ASTUtils.generateVarRef(port);
        return CReactionGenerator.generateDelayBody(
            ref,
            action.getName(),
            CUtil.isTokenType(getInferredType(action), types)
        );
    }


}
