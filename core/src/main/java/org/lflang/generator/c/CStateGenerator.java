package org.lflang.generator.c;

import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ModeInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.StateVar;

public class CStateGenerator {
  /**
   * Generate code for state variables of a reactor in the form "stateVar.type stateVar.name;"
   *
   * @param reactor {@link TypeParameterizedReactor}
   * @param types A helper object for types
   */
  public static String generateDeclarations(TypeParameterizedReactor reactor, CTypes types) {
    CodeBuilder code = new CodeBuilder();
    for (StateVar stateVar : ASTUtils.allStateVars(reactor.reactor())) {
      code.prSourceLineNumber(stateVar);
      code.pr(
          types.getTargetType(reactor.resolveType(ASTUtils.getInferredType(stateVar)))
              + " "
              + stateVar.getName()
              + ";");
    }
    return code.toString();
  }

  /**
   * If the state is initialized with a parameter, then do not use a temporary variable. Otherwise,
   * do, because static initializers for arrays and structs have to be handled this way, and there
   * is no way to tell whether the type of the array is a struct.
   *
   * @param instance {@link ReactorInstance}
   * @param stateVar {@link StateVar}
   * @param mode {@link ModeInstance}
   * @return String
   */
  public static String generateInitializer(
      ReactorInstance instance,
      String selfRef,
      StateVar stateVar,
      ModeInstance mode,
      CTypes types) {
    var initExpr = getInitializerExpr(stateVar, instance);
    String baseInitializer =
        generateBaseInitializer(instance.tpr, selfRef, stateVar, initExpr, types);
    String modalReset = generateModalReset(instance, selfRef, stateVar, initExpr, mode, types);
    return String.join("\n", baseInitializer, modalReset);
  }

  private static String generateBaseInitializer(
      TypeParameterizedReactor tpr,
      String selfRef,
      StateVar stateVar,
      String initExpr,
      CTypes types) {
    if (ASTUtils.isOfTimeType(stateVar)
        || ASTUtils.isParameterized(stateVar) && !stateVar.getInit().getExprs().isEmpty()) {
      return selfRef + "->" + stateVar.getName() + " = " + initExpr + ";";
    } else {
      var declaration =
          types.getVariableDeclaration(tpr, ASTUtils.getInferredType(stateVar), "_initial", true);
      return String.join(
          "\n",
          "{ // For scoping",
          "    static " + declaration + " = " + initExpr + ";",
          "    " + selfRef + "->" + stateVar.getName() + " = _initial;",
          "} // End scoping.");
    }
  }

  private static String generateModalReset(
      ReactorInstance instance,
      String selfRef,
      StateVar stateVar,
      String initExpr,
      ModeInstance mode,
      CTypes types) {
    if (mode == null || !stateVar.isReset()) {
      return "";
    }
    var modeRef =
        "&"
            + CUtil.reactorRef(mode.getParent())
            + "->_lf__modes["
            + mode.getParent().modes.indexOf(mode)
            + "]";
    var type = types.getTargetType(instance.tpr.resolveType(ASTUtils.getInferredType(stateVar)));

    if (ASTUtils.isOfTimeType(stateVar)
        || ASTUtils.isParameterized(stateVar) && !stateVar.getInit().getExprs().isEmpty()) {
      return CModesGenerator.generateStateResetStructure(
          instance, modeRef, selfRef, stateVar.getName(), initExpr, type);
    } else {
      CodeBuilder code = new CodeBuilder();
      var source = "_initial";
      var declaration =
          types.getVariableDeclaration(
              instance.tpr, ASTUtils.getInferredType(stateVar), source, true);
      code.pr("{ // For scoping");
      code.indent();
      code.pr("static " + declaration + " = " + initExpr + ";");
      code.pr(
          CModesGenerator.generateStateResetStructure(
              instance, modeRef, selfRef, stateVar.getName(), source, type));
      code.unindent();
      code.pr("} // End scoping.");
      return code.toString();
    }
  }

  /**
   * Return a C expression that can be used to initialize the specified state variable within the
   * specified parent. If the state variable initializer refers to parameters of the parent, then
   * those parameter references are replaced with accesses to the self struct of the parent.
   */
  private static String getInitializerExpr(StateVar state, ReactorInstance parent) {
    var ctypes = CTypes.generateParametersIn(parent);
    return ctypes.getTargetInitializer(state.getInit(), state.getType());
  }
}
