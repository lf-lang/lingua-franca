package org.lflang.generator.python;

import org.lflang.ast.ASTUtils;
import org.lflang.generator.c.CDelayBodyGenerator;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Reaction;
import org.lflang.lf.VarRef;

public class PythonDelayBodyGenerator extends CDelayBodyGenerator {

  public PythonDelayBodyGenerator(PythonTypes types) {
    super(types);
  }

  /**
   * Generate code for the body of a reaction that takes an input and schedules an action with the
   * value of that input.
   *
   * @param action The action to schedule
   * @param port The port to read from
   */
  @Override
  public String generateDelayBody(Action action, VarRef port) {
    boolean isTokenType = CUtil.isTokenType(ASTUtils.getInferredType(action), types);
    String ref = ASTUtils.generateVarRef(port);
    // Note that the action.type set by the base class is actually
    // the port type.
    if (isTokenType) {
      return String.join(
          "\n",
          "if (" + ref + "->is_present) {",
          "    // Put the whole token on the event queue, not just the payload.",
          "    // This way, the length and element_size are transported.",
          "    lf_schedule_token(" + action.getName() + ", 0, " + ref + "->token);",
          "}");
    } else {
      var value = "self->_lf_" + ref + "->value";
      return String.join(
          "\n",
          "// Create a token.",
          "#if !defined(LF_SINGLE_THREADED)",
          "// Need to lock the mutex first.",
          "lf_mutex_lock(&self->base.environment->mutex);",
          "#endif",
          "lf_token_t* t = _lf_new_token((token_type_t*)"
              + action.getName()
              + ", "
              + value
              + ", 1);",
          "Py_INCREF(" + value + ");",
          "#if !defined(LF_SINGLE_THREADED)",
          "lf_mutex_unlock(&self->base.environment->mutex);",
          "#endif",
          "",
          "// Pass the token along",
          "lf_schedule_token(" + action.getName() + ", 0, t);");
    }
  }

  /**
   * Generate code for the body of a reaction that is triggered by the given action and writes its
   * value to the given port. This realizes the receiving end of a logical delay specified with the
   * 'after' keyword.
   *
   * @param action The action that triggers the reaction
   * @param port The port to write to.
   */
  @Override
  public String generateForwardBody(Action action, VarRef port) {
    String outputName = ASTUtils.generateVarRef(port);
    if (CUtil.isTokenType(ASTUtils.getInferredType(action), types)) {
      return super.generateForwardBody(action, port);
    } else {
      return "lf_set(" + outputName + ", " + action.getName() + "->token->value);";
    }
  }

  @Override
  public void finalizeReactions(Reaction delayReaction, Reaction forwardReaction) {
    ASTUtils.addReactionAttribute(delayReaction, "_c_body");
    ASTUtils.addReactionAttribute(forwardReaction, "_c_body");
  }
}
