package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import org.lflang.InferredType;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;

public class PythonStateGenerator {
  /**
   * Generate state variable instantiations for reactor "decl"
   *
   * @param decl The reactor declaration to generate state variables.
   */
  public static String generatePythonInstantiations(ReactorDecl decl) {
    List<String> lines = new ArrayList<>();
    lines.add("# Define state variables");
    // Next, handle state variables
    for (StateVar state : ASTUtils.allStateVars(ASTUtils.toDefinition(decl))) {
      lines.add("self." + state.getName() + " = " + generatePythonInitializer(state));
    }
    lines.add("");
    return String.join("\n", lines);
  }

  /**
   * Handle initialization for state variable
   *
   * @param state a state variable
   */
  public static String generatePythonInitializer(StateVar state) {
    if (!ASTUtils.isInitialized(state)) {
      return "None";
    }
    return PythonTypes.getInstance()
        .getTargetExpr(state.getInit().getExpr(), InferredType.undefined());
  }
}
