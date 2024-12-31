package org.lflang.generator.c;

import java.util.HashSet;
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
   * @param suppressLineDirectives Whether to suppress the generation of line directives.
   */
  public static String generateDeclarations(
      TypeParameterizedReactor reactor, CTypes types, boolean suppressLineDirectives) {
    CodeBuilder code = new CodeBuilder();
    // Allow derived classes to override base class parameter definitions.
    // Assume that the validator has checked that types match.
    var declared = new HashSet<String>();
    for (Parameter parameter : ASTUtils.allParameters(reactor.reactor())) {
      // If the parameter name has been seen already, assume it is an override of the default value
      // in a derived class.  The validator should check.
      if (declared.contains(parameter.getName())) continue;
      declared.add(parameter.getName());
      code.prSourceLineNumber(parameter, suppressLineDirectives);
      code.pr(
          types.getTargetType(reactor.resolveType(ASTUtils.getInferredType(parameter)))
              + " "
              + parameter.getName()
              + ";");
    }
    code.prEndSourceLineNumber(suppressLineDirectives);
    return code.toString();
  }
}
