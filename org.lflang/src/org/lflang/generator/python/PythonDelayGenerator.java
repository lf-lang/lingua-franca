package org.lflang.generator.python;

import org.lflang.ASTUtils;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;
import org.lflang.generator.c.CDelayGenerator;

public class PythonDelayGenerator extends CDelayGenerator {

    private PythonTypes types;

    public PythonDelayGenerator(PythonTypes types) {
        super(types);
        this.types = types;
    }

    /**
     * Generate code for the body of a reaction that is triggered by the
     * given action and writes its value to the given port. This realizes
     * the receiving end of a logical delay specified with the 'after'
     * keyword.
     *
     * @param action The action that triggers the reaction
     * @param port   The port to write to.
     */
    @Override
    public String generateForwardBody(Action action, VarRef port) {
        String outputName = ASTUtils.generateVarRef(port);
        if (CUtil.isTokenType(ASTUtils.getInferredType(action), types)) {
            return super.generateForwardBody(action, port);
        } else {
            return "lf_set(" + outputName + ", " + action.getName()
                + "->token->value);";
        }
    }

    /**
     * Generate code for the body of a reaction that takes an input and
     * schedules an action with the value of that input.
     *
     * @param action The action to schedule
     * @param port   The port to read from
     */
    @Override
    public String generateDelayBody(Action action, VarRef port) {
        return PythonReactionGenerator.generateCDelayBody(action, port, CUtil.isTokenType(ASTUtils.getInferredType(action), types));
    }
}
