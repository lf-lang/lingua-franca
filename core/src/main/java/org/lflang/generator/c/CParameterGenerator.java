package org.lflang.generator.c;

import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ParameterInstance;
import org.lflang.lf.Initializer;
import org.lflang.lf.Parameter;

/**
 * Generates C code to declare and initialize parameters.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
 */
public class CParameterGenerator {
  /**
   * Return a C expression that can be used to initialize the specified parameter instance. If the
   * parameter initializer refers to other parameters, then those parameter references are replaced
   * with accesses to the self struct of the parents of those parameters.
   */
  public static String getInitializer(ParameterInstance p) {
    // Handle the bank_index parameter.
    if (p.getName().equals("bank_index") && p.getOverride() == null) {
      return CUtil.bankIndex(p.getParent());
    }

    CTypes ctypes = CTypes.generateParametersIn(p.getParent().getParent());
    Initializer values = p.getActualValue();
    return ctypes.getTargetInitializer(values, p.getDefinition().getType());
  }

  /**
   * Generate code for parameters variables of a reactor in the form "parameter.type
   * parameter.name;"
   *
   * @param reactor {@link TypeParameterizedReactor}
   * @param types A helper class for types
   */
  public static String generateDeclarations(TypeParameterizedReactor reactor, CTypes types) {
    CodeBuilder code = new CodeBuilder();
    for (Parameter parameter : ASTUtils.allParameters(reactor.reactor())) {
      code.prSourceLineNumber(parameter);
      code.pr(
          types.getTargetType(reactor.resolveType(ASTUtils.getInferredType(parameter)))
              + " "
              + parameter.getName()
              + ";");
    }
    return code.toString();
  }
}
